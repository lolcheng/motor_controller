import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from encos_msgs.action import RunExperiment
from encos_msgs.msg import MotorCommand, MotorState
from encos_msgs.srv import Arm
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy


VIRTUAL_QOS = QoSProfile(depth=8, reliability=ReliabilityPolicy.BEST_EFFORT)


def tool(package, filename):
    return Path(get_package_prefix(package)) / 'lib' / package / filename


class Process:
    def __init__(self, name, cmd, directory):
        self.log = (directory / f'{name}.log').open('w')
        self.process = subprocess.Popen([str(part) for part in cmd], stdout=self.log, stderr=subprocess.STDOUT)
        self.directory = directory

    def stop(self, signum=signal.SIGINT):
        if self.process.poll() is None:
            self.process.send_signal(signum)
        try:
            return self.process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=3)
            raise RuntimeError(f'process did not stop: {self.process.args}')
        finally:
            self.log.close()


class Rig:
    def __init__(self, fmt, directory, driver_path, hz=400, faults=(), raw=True, writes=True, modes=True):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.interface = os.environ.get('ENCOS_TEST_VCAN', 'vcan0')
        self.children = []
        self.driver_path = driver_path
        self.peer = self.spawn('virtual_motor', [tool('encos_tools', 'encos_virtual_motor'), self.interface, fmt,
                               '--bootstrap-beacon', *[item for fault in faults for item in ('--fault', fault)]])
        self.driver = self.spawn('driver', [driver_path, '--ros-args',
            '-p', f'can_interface:={self.interface}', '-p', f"fd_brs:={'true' if fmt == 'fd' else 'false'}",
            '-p', f'control_hz:={hz}', '-p', f'publish_raw:={str(raw).lower()}',
            '-p', f'allow_virtual_writes:={str(writes).lower()}',
            *[item for key in ('current_torque', 'braking', 'electromagnetic_brake')
              for item in ('-p', f'allow_virtual_{key}:={str(modes).lower()}')]])
        self.node = rclpy.create_node('encos_p2_test')
        self.state = None
        self.state_sub = self.node.create_subscription(MotorState, '/encos/motor_state', self.on_state, VIRTUAL_QOS)
        self.publisher = self.node.create_publisher(MotorCommand, '/encos/motor_command', VIRTUAL_QOS)
        self.client_cache = {}
        self.action = ActionClient(self.node, RunExperiment, '/encos/run_experiment')

    def on_state(self, value):
        self.state = value

    def spawn(self, name, args):
        child = Process(name, args, self.directory)
        self.children.append(child)
        return child

    def until(self, condition, timeout=5):
        end = time.monotonic() + timeout
        while not condition():
            if self.driver.process.poll() is not None:
                raise AssertionError(f'driver exited {self.driver.process.returncode}: {self.directory}')
            if time.monotonic() >= end:
                raise AssertionError(f'timed out: {self.directory}')
            rclpy.spin_once(self.node, timeout_sec=.01)

    def service(self, name, kind, **values):
        service = self.client_cache.get(name)
        if service is None:
            service = self.node.create_client(kind, f'/encos/{name}')
            self.client_cache[name] = service
        assert service.wait_for_service(timeout_sec=7), name
        future = service.call_async(kind.Request(**values))
        self.until(future.done, timeout=10)
        return future.result()

    def arm(self, authority=Arm.Request.TOPIC, owner=17):
        result = self.service('arm', Arm, authority=authority, owner=owner, confirm=True)
        assert result.success, result.detail
        self.until(lambda: self.state is not None and self.state.authority == authority)

    def goal(self, goal):
        assert self.action.wait_for_server(timeout_sec=5)
        result = self.action.send_goal_async(goal)
        self.until(result.done)
        return result.result()

    def publish(self, value, publisher=None):
        value.header.stamp = self.node.get_clock().now().to_msg()
        (publisher or self.publisher).publish(value)

    def readiness(self):
        self.until(lambda: self.state is not None and self.state.connected, timeout=8)
        self.until(lambda: self.publisher.get_subscription_count() == 1)

    def close(self):
        self.action.destroy()
        self.node.destroy_node()
        for child in reversed(self.children):
            child.stop()

    def __enter__(self):
        return self

    def __exit__(self, _type, _value, _traceback):
        self.close()


def position(owner=17, value=.05, feedback=2, ttl_ns=100_000_000):
    command = MotorCommand(mode=MotorCommand.POSITION, motor_id=1, owner=owner, feedback=feedback,
                           speed_limit_rad_s=.5, current_limit_a=1., position_rad=value)
    command.valid_for.nanosec = ttl_ns
    return command


def action_goal(owner=17, duration=1., shape=0):
    return RunExperiment.Goal(owner=owner, mode=MotorCommand.POSITION, shape=shape, duration_s=float(duration),
                               offset=.05, speed_limit_rad_s=.5, current_limit_a=1., feedback=1)
