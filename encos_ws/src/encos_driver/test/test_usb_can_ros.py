#!/usr/bin/env python3
"""Validate the USB-CAN ROS interface against a Linux pseudo-terminal."""

from contextlib import contextmanager
import math
import os
import pty
import select
import signal
import subprocess
import sys
import time

import rclpy
from encos_msgs.msg import MotorState, RawFrame
from encos_msgs.srv import DiscoverMotor, ReadParameter
from rclpy.qos import qos_profile_sensor_data


@contextmanager
def driver(allow_queries):
    master, slave = pty.openpty()
    process = subprocess.Popen([sys.argv[1], '--ros-args', '-p', f'device:={os.ttyname(slave)}',
                                '-p', f'allow_queries:={str(allow_queries).lower()}'])
    try:
        yield master, process
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        os.close(master)
        os.close(slave)


def spin_until(node, condition, process, timeout=5):
    end = time.monotonic() + timeout
    while not condition():
        assert process.poll() is None and time.monotonic() < end
        rclpy.spin_once(node, timeout_sec=.02)


def read_packet(fd, length):
    result = bytearray()
    deadline = time.monotonic() + 3
    while len(result) < length:
        assert select.select([fd], [], [], max(0, deadline - time.monotonic()))[0]
        result.extend(os.read(fd, length - len(result)))
    return result


def feed_until(node, fd, packet, received, process):
    deadline = time.monotonic() + 5
    while not received():
        assert process.poll() is None and time.monotonic() < deadline
        os.write(fd, packet)
        rclpy.spin_once(node, timeout_sec=.1)


def main():
    rclpy.init()
    node = rclpy.create_node('test_usb_can')
    frames, states = [], []
    node.create_subscription(RawFrame, '/encos/raw_rx', frames.append, qos_profile_sensor_data)
    node.create_subscription(MotorState, '/encos/motor_state', states.append, qos_profile_sensor_data)
    discover = node.create_client(DiscoverMotor, '/encos/discover_motor')
    read = node.create_client(ReadParameter, '/encos/read_parameter')
    try:
        with driver(False) as (master, process):
            spin_until(node, lambda: node.count_publishers('/encos/raw_rx') > 0, process)
            feed_until(node, master, bytes.fromhex('AA 00 01 0D 00 00 00 00 00 00 00 00 B8'),
                       lambda: frames, process)
            assert frames[-1].id == 1 and frames[-1].length == 8 and not frames[-1].kernel_realtime_valid
            assert not states and not discover.service_is_ready()
            assert not select.select([master], [], [], .05)[0]

        with driver(True) as (master, process):
            spin_until(node, lambda: discover.service_is_ready() and read.service_is_ready(), process)
            assert not select.select([master], [], [], .05)[0]
            future = discover.call_async(DiscoverMotor.Request())
            assert read_packet(master, 9) == bytes.fromhex('AA 07 FF 09 FF FF 00 82 39')
            os.write(master, bytes.fromhex('AA 07 FF 0A FF FF 01 00 01 BA'))
            spin_until(node, future.done, process)
            assert future.result().success and future.result().motor_id == 1

            feed_until(node, master, bytes.fromhex('AA 00 01 0D 40 00 00 00 00 00 00 64 5C'),
                       lambda: states, process)
            future = read.call_async(ReadParameter.Request(code=1))
            assert read_packet(master, 7) == bytes.fromhex('AA 00 01 07 E1 01 94')
            os.write(master, bytes.fromhex('AA 00 01 0B A0 01 42 B4 00 00 4D'))
            spin_until(node, lambda: future.done() and math.isclose(states[-1].position_rad, math.pi / 2), process)
            assert future.result().success and math.isclose(future.result().value.values[0], math.pi / 2)
            assert states[-1].valid[0] and math.isclose(states[-1].position_rad, math.pi / 2)
            assert not states[-1].valid[1] and not states[-1].virtual_device
            assert not states[-1].kernel_rx_valid and states[-1].safety_state == MotorState.READ_ONLY

            repeated = read.call_async(ReadParameter.Request(code=1))
            spin_until(node, repeated.done, process)
            assert not repeated.result().success and not select.select([master], [], [], .05)[0]
            print('USB-CAN PTY ROS passive monitoring, ID/position query, state and send guard PASS')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
