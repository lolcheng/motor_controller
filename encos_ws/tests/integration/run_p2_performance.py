#!/usr/bin/env python3
"""Run the Jazzy/vcan P2 measurement matrix and retain raw DDS and bag samples."""
import argparse
import csv
import json
from pathlib import Path
import signal
import sqlite3
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from encos_msgs.msg import CommunicationStats, ControlSample, MotorFault, MotorState, RawFrame
from encos_msgs.srv import Arm, GetProfile, Stop

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'src/encos_driver/test'), str(ROOT / 'src/encos_experiments/scripts')]
from ros_harness import Rig, VIRTUAL_QOS, action_goal, tool
from analyze_timing import summarize
from export_samples import export, sample_row


STATS = ('tx', 'rx', 'matched', 'expected_replies', 'timeouts', 'invalid_rx', 'ignored_rx',
         'command_received', 'dropped_commands', 'expired_commands',
         'skipped_cycles', 'overruns', 'output_overflows', 'socket_rx_drops')


def cpu_seconds(process):
    pid = process.process.pid
    tail = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
    import os
    return (int(tail[11]) + int(tail[12])) / os.sysconf('SC_CLK_TCK')


def snapshot_counts(stats):
    return {name: int(getattr(stats[-1], name)) for name in STATS} if stats else {name: 0 for name in STATS}


def run(fmt, scenario, hz, duration, directory, driver):
    with Rig(fmt, directory, driver, hz=hz, raw=scenario in 'DE', writes=False, modes=False) as rig:
        rig.readiness()
        samples, monitor, all_stats, faults = [], {}, [], []
        rig.node.create_subscription(ControlSample, '/encos/control_sample', samples.append, 4096)
        rig.node.create_subscription(CommunicationStats, '/encos/communication_stats', all_stats.append, 20)
        rig.node.create_subscription(MotorFault, '/encos/fault', faults.append, 20)
        bag = None
        count_diag = [0]
        if scenario in 'BDE':
            def observed(topic):
                def receive(_):
                    monitor[topic] = monitor.get(topic, 0) + 1
                return receive
            rig.node.create_subscription(MotorState, '/encos/motor_state', observed('state'), VIRTUAL_QOS)
            if scenario in 'DE':
                rig.node.create_subscription(RawFrame, '/encos/raw_rx', observed('raw_rx'), VIRTUAL_QOS)
                rig.node.create_subscription(RawFrame, '/encos/raw_tx', observed('raw_tx'), VIRTUAL_QOS)
        if scenario in 'CD':
            topics = ['/encos/control_sample', '/encos/motor_state', '/encos/communication_stats', '/encos/fault']
            if scenario == 'D':
                topics += ['/encos/raw_rx', '/encos/raw_tx']
            bag = rig.spawn('bag', ['ros2', 'bag', 'record', '-s', 'sqlite3',
                            '--disable-keyboard-controls', '-o', directory / 'bag', '--topics', *topics])
            rig.until(lambda: rig.node.count_subscribers('/encos/control_sample') >= 2, 12)

        sent = [0]
        noise = None
        peers = {'driver': rig.driver, 'virtual_motor': rig.peer}
        if bag:
            peers['rosbag'] = bag
        if scenario == 'E':
            ping = rig.node.create_client(GetProfile, '/encos/get_profile')
            pending = []
            def load():
                pending[:] = [item for item in pending if not item.done()]
                if len(pending) < 4:
                    pending.append(ping.call_async(GetProfile.Request()))
                    count_diag[0] += 1
            noise = rig.node.create_timer(.002, load)

        if scenario != 'A':
            stop_client = rig.node.create_client(Stop, '/encos/stop')
            if not stop_client.wait_for_service(timeout_sec=5):
                raise RuntimeError('stop service unavailable before measurement')
            rig.client_cache['stop'] = stop_client
        rig.arm(Arm.Request.ACTION if scenario == 'A' else Arm.Request.TOPIC)
        rig.until(lambda: all_stats)
        start_counts = snapshot_counts(all_stats)
        before_cpu = {name: cpu_seconds(child) for name, child in peers.items()}
        start = time.monotonic()
        result = None
        if scenario == 'A':
            handle = rig.goal(action_goal(duration=duration))
            if not handle.accepted:
                raise RuntimeError('action admission failed')
            future = handle.get_result_async()
            rig.until(future.done, duration+4)
            result = future.result()
        else:
            source = rig.spawn('topic_source', [tool('encos_experiments', 'encos_topic_source'),
                         17, hz * 2, duration, directory / 'source.csv'])
            rig.until(lambda: source.process.poll() is not None, duration+6)
            if source.process.returncode != 0:
                raise RuntimeError('C++ topic source failed')
            rig.service('stop', Stop)
            with (directory / 'source.csv').open(newline='') as source_file:
                source_times = [int(row['publish_mono_ns']) for row in csv.DictReader(source_file)]
            sent[0] = len(source_times)
        elapsed = time.monotonic() - start
        after_cpu = {name: cpu_seconds(child) for name, child in peers.items()}
        if noise:
            noise.cancel()
        rig.until(lambda: rig.state and not rig.state.authority and
                  rig.state.safety_state in (MotorState.UNARMED, MotorState.FAULT, MotorState.STOPPING), 4)
        tail_end = time.monotonic() + .65
        rig.until(lambda: time.monotonic() >= tail_end, 2)

        rows = [sample_row(value) for value in samples]
        if not rows:
            raise RuntimeError('no ROS control samples')
        with (directory / 'samples.csv').open('x', newline='') as output:
            writer = csv.DictWriter(output, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        summary = summarize(rows, hz)
        summary.update(format=fmt, scenario=scenario, planned_duration_s=duration, run_wall_s=elapsed,
                       command_pub_count=sent[0], command_pub_hz=sent[0]/elapsed if sent[0] else None,
                       observed_ros=monitor, diagnostic_calls=count_diag[0],
                       cpu_percent={name: 100*(after_cpu[name]-before_cpu[name])/elapsed for name in peers},
                       driver_stats_delta={name: val-start_counts[name] for name,val in snapshot_counts(all_stats).items()},
                       final_safety=int(rig.state.safety_state), action_status=result.status if result else None,
                       faults=[{'state': f.safety_state, 'code': f.software_fault, 'detail': f.detail}
                               for f in faults if f.latched])
        if scenario != 'A' and len(source_times) > 1:
            deltas = [(b-a) / 1e6 for a,b in zip(source_times,source_times[1:])]
            summary['source_max_gap_ms'] = max(deltas)
            summary['command_pub_hz'] = (len(source_times)-1)*1e9/(source_times[-1]-source_times[0])
        if result:
            summary['action_samples'] = result.result.samples
        expected_ok = ((result is None or result.status == GoalStatus.STATUS_SUCCEEDED) and
                       summary['final_safety'] == MotorState.UNARMED)
        full_duration = summary['tx_span_s'] >= max(duration - .25, 60. if duration >= 61 else 0.)
        summary['status'] = 'PASS' if expected_ok and full_duration and summary['timeouts']==0 and not summary['sequence_gaps'] and (
                            hz < 400 or summary['feedback_hz'] >= .99 * hz) else 'FAIL'
        if bag:
            bag.stop()
            count = export(directory / 'bag', directory / 'bag_samples.csv')
            with (directory / 'bag_samples.csv').open(newline='') as source:
                bag_ids = [int(row['sequence']) for row in csv.DictReader(source)]
            database = (directory / 'bag' / 'bag_0.db3').resolve()
            with sqlite3.connect(database.as_uri() + '?mode=ro', uri=True) as conn:
                db_count = conn.execute('SELECT count(*) FROM messages JOIN topics ON messages.topic_id=topics.id '
                                        "WHERE topics.name='/encos/control_sample'").fetchone()[0]
            sample_ids = [row['sequence'] for row in rows]
            summary['bag_control_samples']=count
            summary['bag_missing_samples']=len(set(sample_ids) - set(bag_ids))
            summary['bag_database_count']=db_count
            if bag_ids != sample_ids or len(set(bag_ids)) != db_count or len(bag_ids) != count:
                summary['status'] = 'FAIL'
        (directory/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        print(json.dumps({key: summary[key] for key in ('format','scenario','target_hz','feedback_hz','timeouts','status')}),
              flush=True)
        return summary['status']=='PASS'


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('format',choices=['classic','fd'])
    p.add_argument('output',type=Path)
    p.add_argument('--scenarios',default='ABCDE')
    p.add_argument('--rates',default='10,50,100,200,400')
    p.add_argument('--seconds-400',type=float,default=61.)
    p.add_argument('--short-seconds',type=float,default=2.5)
    p.add_argument('--driver',type=Path)
    args=p.parse_args()
    if not args.scenarios or any(c not in 'ABCDE' for c in args.scenarios):
        p.error('invalid scenarios')
    args.output.mkdir(parents=True,exist_ok=False)
    driver=args.driver or tool('encos_driver','encos_driver_node')
    failures=0
    rclpy.init()
    try:
        for hz in map(int,args.rates.split(',')):
            for scenario in args.scenarios:
                directory=args.output/f'{hz}hz_{scenario}'
                try:
                    if not run(args.format,scenario,hz,args.seconds_400 if hz==400 else args.short_seconds,
                               directory,driver):
                        failures+=1
                except Exception as error:
                    failures+=1
                    directory.mkdir(parents=True,exist_ok=True)
                    (directory/'failure.txt').write_text(f'{type(error).__name__}: {error}\n')
                    print(f'{hz}Hz {scenario} FAIL: {error}',flush=True)
    finally:
        rclpy.shutdown()
    return 1 if failures else 0


if __name__=='__main__':
    sys.exit(main())
