#!/usr/bin/env python3
"""Publish a finite, timestamped Topic command sequence for the vcan simulator."""
import argparse
import math
import time

import rclpy
from encos_msgs.msg import MotorCommand
from encos_msgs.srv import Arm, Stop
from rclpy.qos import QoSProfile, ReliabilityPolicy

MODES = ('mixed', 'position', 'velocity', 'current', 'torque', 'damping',
         'dissipative', 'regenerative', 'electromagnetic-brake')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode', choices=MODES)
    p.add_argument('value', type=float)
    p.add_argument('--owner', required=True, type=int)
    p.add_argument('--id', default=1, type=int)
    p.add_argument('--rate', default=50, type=int)
    p.add_argument('--seconds', default=2.0, type=float)
    p.add_argument('--feedback', default=1, type=int, choices=range(4))
    p.add_argument('--arm-virtual', action='store_true')
    p.add_argument('--stop-on-exit', action='store_true')
    args = p.parse_args()
    if not 0 < args.rate <= 1000 or not 0 < args.seconds <= 90 or not math.isfinite(args.value):
        p.error('invalid rate, duration or target')
    rclpy.init()
    node = rclpy.create_node('encos_virtual_control')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    publisher = node.create_publisher(MotorCommand, '/encos/motor_command', qos)
    try:
        stop = node.create_client(Stop, '/encos/stop') if args.stop_on_exit else None
        if stop and not stop.wait_for_service(timeout_sec=3):
            raise RuntimeError('stop service unavailable')
        if args.arm_virtual:
            arm = node.create_client(Arm, '/encos/arm')
            if not arm.wait_for_service(timeout_sec=3):
                raise RuntimeError('driver unavailable')
            future = arm.call_async(Arm.Request(authority=Arm.Request.TOPIC, owner=args.owner, confirm=True))
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            if not future.done() or not future.result().success:
                raise RuntimeError('virtual authorization refused')
        m = MotorCommand(motor_id=args.id, owner=args.owner, mode=MODES.index(args.mode),
                         feedback=args.feedback, speed_limit_rad_s=0.5, current_limit_a=1.0)
        m.valid_for.nanosec = 100_000_000
        if m.mode in (m.POSITION, m.MIXED):
            m.position_rad = args.value
        if m.mode == m.MIXED:
            m.kp, m.kd = 4.0, .1
        if m.mode == m.VELOCITY:
            m.velocity_rad_s = args.value
        if m.mode == m.CURRENT:
            m.current_a = args.value
        if m.mode == m.TORQUE:
            m.torque_nm = args.value
        if m.mode in (m.DISSIPATIVE, m.REGENERATIVE):
            m.current_limit_a = args.value
        if m.mode == m.ELECTROMAGNETIC_BRAKE:
            m.release_brake = bool(args.value)
        start, count = time.monotonic(), 0
        total = math.ceil(args.rate * args.seconds)
        while count < total:
            if not rclpy.ok():
                break
            time.sleep(max(0, start + count / args.rate - time.monotonic()))
            index = max(count, int((time.monotonic() - start) * args.rate))
            if index >= total:
                break
            m.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(m)
            count = index + 1
        if stop and rclpy.ok():
            result = stop.call_async(Stop.Request())
            rclpy.spin_until_future_complete(node, result, timeout_sec=2)
            if not result.done() or not result.result().success:
                raise RuntimeError('stop request failed')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
