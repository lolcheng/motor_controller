#!/usr/bin/env python3
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from encos_msgs.msg import ControlSample, MotorCommand, MotorFault, MotorState, ParameterValue, RawFrame
from encos_msgs.srv import (Arm, DiscoverMotor, GetProfile, ReadParameter, ResetFault, ResetMotorId,
                            SetBrake, SetMotorId, SetZero, Stop, WriteParameter)
from ros_harness import Rig, VIRTUAL_QOS, action_goal, position, tool
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def ok(response):
    assert response.success, response.detail
    return response


def read(rig, code, group=0):
    return ok(rig.service('read_parameter', ReadParameter, code=code, group=group))


def stop(rig):
    ok(rig.service('stop', Stop))
    rig.until(lambda: rig.state and rig.state.authority == 0 and rig.state.safety_state == MotorState.UNARMED)


def write_tests(rig):
    inputs = {
        4: (ParameterValue.SCALAR, [2.6], 22),
        5: (ParameterValue.PAIR, [0., 600.], 23),
        6: (ParameterValue.PAIR, [0., 10.], 24),
        7: (ParameterValue.PAIR, [-12., 12.], 25),
        8: (ParameterValue.PAIR, [-17., 17.], 26),
        9: (ParameterValue.PAIR, [-35., 35.], 27),
        10: (ParameterValue.PAIR, [-80., 80.], 28),
        11: (ParameterValue.SCALAR, [480.], 31),
        12: (ParameterValue.PAIR, [.02, 60.], 32),
        13: (ParameterValue.PAIR, [.007, .08], 33),
        14: (ParameterValue.PAIR, [.008, .00005], 34),
        17: (ParameterValue.SCALAR, [math.radians(-10.)], 38),
    }
    for code, (kind, numbers, query) in inputs.items():
        values = numbers + [0.] * (4 - len(numbers))
        response = ok(rig.service('write_parameter', WriteParameter, code=code,
                value=ParameterValue(kind=kind, values=values), confirm=True))
        assert response.acknowledged and response.raw.data[2] == code
        result = read(rig, query)
        assert result.value.kind == kind and all(math.isclose(a, b, abs_tol=1e-8)
            for a,b in zip(result.value.values, numbers))
    for group in range(1,11):
        response = ok(rig.service('write_parameter', WriteParameter, code=16,
            value=ParameterValue(kind=ParameterValue.KT_GROUP, group=group, values=[1., 2., 3., 4.]), confirm=True))
        assert response.readback.group == group and list(response.readback.values) == [1., 2., 3., 4.]
    for code in (1, 2):
        response = rig.service('write_parameter', WriteParameter, code=code, confirm=True)
        assert not response.success and not response.acknowledged and 'ambiguity' in response.detail
    response = rig.service('write_parameter', WriteParameter, code=11,
        value=ParameterValue(kind=ParameterValue.SCALAR), confirm=True)
    assert not response.success and 'timeout' in response.detail


def functional(fmt, root, driver):
    with Rig(fmt, root / fmt, driver) as rig:
        samples, raw_tx = [], []
        rig.node.create_subscription(ControlSample, '/encos/control_sample', samples.append, 1000)
        rig.node.create_subscription(RawFrame, '/encos/raw_tx', raw_tx.append, VIRTUAL_QOS)
        rig.readiness()
        assert not raw_tx and not rig.state.profile_verified
        d = ok(rig.service('discover_motor', DiscoverMotor))
        assert d.motor_id == 1 and bytes(d.raw.data[:d.raw.length]) == bytes.fromhex('ffff010001')
        assert d.raw.brs == (fmt == 'fd')
        assert not rig.service('arm', Arm, authority=0, owner=1, confirm=True).success
        ok(rig.service('arm', Arm, authority=Arm.Request.READ_ONLY, confirm=True))
        for q in list(range(1,6)) + list(range(22,40)):
            reply = read(rig, q, 2 if q == 36 else 0)
            if q == 29:
                assert bytes(reply.value.bytes) == b'ENCOS\x02\x00\x01\x02\x03\x04\x05'
            if q == 5:
                assert reply.value.kind == ParameterValue.RAW16 and reply.value.raw_u16 == 2000
            if q == 39:
                assert reply.value.kind == ParameterValue.ENCODER and math.isnan(reply.value.values[0])
        assert not rig.service('read_parameter', ReadParameter, code=6).success
        write_tests(rig)
        ok(rig.service('set_motor_id', SetMotorId, new_id=0x123, confirm=True))
        assert ok(rig.service('discover_motor', DiscoverMotor)).motor_id == 0x123
        ok(rig.service('reset_motor_id', ResetMotorId, confirm=True))
        assert ok(rig.service('discover_motor', DiscoverMotor)).motor_id == 1
        ok(rig.service('set_zero', SetZero, use_offset=True, offset_rad=math.radians(-10.), confirm=True))
        assert math.isclose(read(rig, 38).value.values[0], math.radians(-10.))
        ok(rig.service('set_zero', SetZero, confirm=True))
        ok(rig.service('set_brake', SetBrake, release=False, confirm=True))
        assert read(rig, 37).value.raw_enum == 0
        ok(rig.service('set_brake', SetBrake, release=True, electromagnetic=True, confirm=True))
        assert read(rig, 37).value.raw_enum == 1

        for mode in range(9):
            rig.arm()
            command = position(value=.025, feedback=3 if mode == MotorCommand.VELOCITY else 2)
            command.mode = mode
            command.velocity_rad_s, command.kp, command.kd = .04, 4., .1
            command.current_a, command.torque_nm = .03, .04
            command.release_brake = True
            before = len(samples)
            rig.publish(command)
            rig.until(lambda: len(samples) > before)
            result = samples[-1]
            assert result.matched, (mode, result.transaction_state)
            assert result.feedback.virtual_device and result.feedback.rx_mono_ns >= result.tx_call_before_mono_ns
            assert result.ros_publish_mono_ns >= result.feedback.rx_userspace_mono_ns
            if mode == MotorCommand.POSITION:
                assert result.feedback.feedback_type == 2 and result.feedback.valid[0] and not result.feedback.valid[1]
                assert math.isnan(result.feedback.velocity_rad_s)
            if mode == MotorCommand.VELOCITY:
                assert result.feedback.feedback_type == 3 and not result.feedback.valid[0] and result.feedback.valid[1]
            if mode == MotorCommand.ELECTROMAGNETIC_BRAKE:
                assert result.feedback.feedback_type == 6
            if mode != MotorCommand.ELECTROMAGNETIC_BRAKE:
                assert not rig.service('write_parameter', WriteParameter, code=4, confirm=True).success
            stop(rig)

        for shape in range(4):
            rig.arm(Arm.Request.ACTION)
            g = action_goal(duration=.31, shape=shape)
            if shape == 1:
                g.mode = MotorCommand.VELOCITY
            if shape:
                g.amplitude, g.step_at_s, g.start_hz, g.end_hz = .015, .13, .8, 1.6
            handle = rig.goal(g)
            assert handle.accepted
            result = handle.get_result_async()
            rig.until(result.done)
            done = result.result()
            assert done.status == GoalStatus.STATUS_SUCCEEDED and done.result.samples > 30
            rig.until(lambda: rig.state.safety_state == MotorState.UNARMED and not rig.state.authority)
        rig.arm(Arm.Request.ACTION)
        assert not rig.goal(action_goal(owner=404)).accepted
        g = rig.goal(action_goal(duration=3))
        assert g.accepted
        rig.until(lambda: rig.state.safety_state == MotorState.RUNNING)
        cancel = g.cancel_goal_async()
        rig.until(cancel.done)
        done = g.get_result_async()
        rig.until(done.done)
        assert done.result().status == GoalStatus.STATUS_CANCELED
        rig.until(lambda: rig.state.safety_state == MotorState.UNARMED and not rig.state.authority)

        rig.arm()
        before = len(samples)
        invalid_stamp = position()
        rig.publisher.publish(invalid_stamp)
        sleep_end = time.monotonic()+.04
        rig.until(lambda: time.monotonic()>sleep_end)
        assert len(samples)==before
        rig.publish(position(ttl_ns=60_000_000))
        rig.until(lambda: len(samples)>before)
        other = rig.node.create_publisher(MotorCommand, '/encos/motor_command', VIRTUAL_QOS)
        rig.until(lambda: other.get_subscription_count()==1)
        rig.publish(position(value=.3), other)
        rig.publish(position(owner=88,value=.4))
        rig.until(lambda: rig.state.safety_state == MotorState.STOPPING, 2)
        ok(rig.service('stop', Stop))  # Stop must not turn an existing fault into a normal disarm.
        rig.until(lambda: rig.state.safety_state == MotorState.FAULT, 3)
        received_faults = []
        rig.node.create_subscription(MotorFault, '/encos/fault', received_faults.append,
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        rig.until(lambda: received_faults and received_faults[-1].latched and
                  received_faults[-1].safety_state == MotorState.FAULT)
        held = samples[before:]
        assert len(held) > 1
        assert len({sample.command_received_mono_ns for sample in held}) == 1
        assert all(sample.command.position_rad == .05 for sample in held)
        assert all(0 <= sample.tx_call_before_mono_ns - sample.command_generated_mono_ns < 60_000_000
                   for sample in held)
        assert not rig.service('reset_fault', ResetFault, confirm=False).success
        ok(rig.service('reset_fault', ResetFault, confirm=True))
        rig.until(lambda: rig.state.safety_state == MotorState.UNARMED)
        assert not rig.state.authority
        for variant in ('nan','inf','magnitude','enum'):
            rig.arm()
            rejected = position(value=float('nan') if variant=='nan' else (1.2 if variant=='magnitude' else .1))
            if variant=='inf':
                rejected.current_limit_a=float('inf')
            if variant=='enum':
                rejected.mode=254
            before=len(samples)
            rig.publish(rejected)
            rig.until(lambda: rig.state.safety_state==MotorState.FAULT,3)
            assert len(samples)==before
            ok(rig.service('reset_fault', ResetFault, confirm=True))
        # A calibrated-write ACK is available; cross-interpretation of query 35 remains blocked.
        r = rig.service('write_parameter', WriteParameter, code=15, confirm=True,
                       value=ParameterValue(kind=ParameterValue.ENUMERATION, raw_enum=2))
        assert r.acknowledged and not r.success and 'isolated' in r.detail
        assert all(x.brs == (fmt == 'fd') for x in raw_tx)
        print(fmt, 'ROS services, all modes, type 1-6, all trajectories, lifecycle and authority PASS', flush=True)


def faults(root, driver):
    for variant, flags in [('hot',('hot',)), ('motor',('motor:2',)), ('uuid_missing',('uuid-missing',)),
                           ('uuid_reverse',('uuid-reverse','duplicate')), ('drops',('drop:30',)),
                           ('delay',('delay:45',))]:
        with Rig('classic', root / variant, driver, faults=flags, writes=False) as rig:
            rig.readiness()
            assert not rig.service('write_parameter', WriteParameter, code=4, confirm=True).success
            if variant in ('hot','motor'):
                response=rig.service('arm', Arm, authority=2, owner=17, confirm=True)
                assert not response.success
                rig.until(lambda: rig.state.safety_state==MotorState.FAULT, 3)
                if variant=='motor':
                    assert rig.state.motor_error==2
                    assert not rig.service('reset_fault', ResetFault, confirm=True).success
            elif variant=='uuid_missing':
                assert not rig.service('read_parameter',ReadParameter,code=29).success
            elif variant=='uuid_reverse':
                assert bytes(read(rig,29).value.bytes).startswith(b'ENCOS')
            else:
                rig.arm(Arm.Request.ACTION)
                handle=rig.goal(action_goal(duration=2))
                assert handle.accepted
                result=handle.get_result_async()
                rig.until(result.done,4)
                assert result.result().status==GoalStatus.STATUS_ABORTED
                rig.until(lambda: rig.state.safety_state==MotorState.FAULT,4)
            print(variant,'ROS fault case PASS',flush=True)


def driver_exit(root, driver):
    for fmt, signum, expected in [('classic', signal.SIGINT, 0), ('fd', signal.SIGKILL, -signal.SIGKILL)]:
        with Rig(fmt, root / f'{fmt}_{signum.name}', driver) as rig:
            rig.readiness()
            rig.arm(Arm.Request.ACTION)
            goal = action_goal(duration=4.)
            goal.mode, goal.offset = MotorCommand.VELOCITY, .15
            assert rig.goal(goal).accepted
            rig.until(lambda: rig.state.safety_state == MotorState.RUNNING and rig.state.valid[1]
                      and rig.state.velocity_rad_s > .05)
            args = rig.driver.process.args
            assert rig.driver.stop(signum) == expected
            time.sleep(.56)
            result = subprocess.run([tool('encos_tools', 'encos_cli'), rig.interface, fmt, 'query', '2'],
                                    check=True, capture_output=True, text=True, timeout=2)
            (rig.directory / 'watchdog.log').write_text(result.stdout)
            assert 'A00200000000' in result.stdout
            rig.driver = rig.spawn('restarted_driver', args)
            rig.state = None
            # SIGKILL leaves the former endpoint in DDS discovery until its lease expires.
            rig.until(lambda: rig.state is not None and rig.state.connected and
                      rig.state.safety_state == MotorState.UNARMED and not rig.state.authority, timeout=8)
            assert read(rig, 2).value.values[0] == 0.
            print(fmt, signum.name, 'driver exit, watchdog and disarmed restart PASS', flush=True)


def main():
    iface=os.environ.get('ENCOS_TEST_VCAN')
    if not iface:
        print('Set ENCOS_TEST_VCAN within a private namespace to run ROS integration')
        return 77
    root=Path(os.environ.get('ENCOS_P2_LOGS','logs/p2/ros-integration')).resolve()
    root.mkdir(parents=True,exist_ok=True)
    rclpy.init()
    try:
        driver=Path(sys.argv[1]).resolve()
        functional('classic',root,driver)
        functional('fd',root,driver)
        faults(root,driver)
        driver_exit(root,driver)
    finally:
        rclpy.shutdown()
    return 0


if __name__=='__main__':
    sys.exit(main())
