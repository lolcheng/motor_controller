#!/usr/bin/env python3
"""Export ENCOS CAN frames, motor state or historical control samples from rosbag2."""
import argparse
import csv
from pathlib import Path


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def measurement_fields(feedback):
    row = {}
    for index, field in enumerate(('position_rad', 'velocity_rad_s', 'phase_current_a', 'estimated_torque_nm',
                                   'motor_temperature_c', 'mos_temperature_c')):
        row[field] = getattr(feedback, field) if feedback.valid[index] else None
        row[f'{field}_valid'] = feedback.valid[index]
        row[f'{field}_source'] = feedback.source[index]
        row[f'{field}_mono_ns'] = feedback.measurement_mono_ns[index] if feedback.valid[index] else None
    return row


def sample_row(sample, bag_ns=None):
    cmd, feedback = sample.command, sample.feedback
    row = {
        'sequence': sample.sequence, 'bag_record_ns': bag_ns,
        'ros_publish_ns': stamp_ns(sample.header.stamp),
        'ros_generation_ns': stamp_ns(cmd.header.stamp) or None,
        'ros_receipt_ns': stamp_ns(sample.ros_command_received) if sample.ros_command_received_valid else None,
        **{field: getattr(sample, field) for field in (
            'command_generated_mono_ns', 'command_received_mono_ns', 'scheduled_mono_ns',
            'cycle_started_mono_ns', 'previous_cycle_interval_ns', 'tx_call_before_mono_ns',
            'tx_call_after_mono_ns', 'rx_parsed_mono_ns', 'ros_publish_mono_ns', 'skipped_cycles',
            'requested_reply', 'matched', 'applied_ack', 'transaction_state', 'send_status')},
        'motor_id': cmd.motor_id, 'owner': cmd.owner, 'mode': cmd.mode, 'requested_ack': cmd.feedback,
        **{f'command_{field}': getattr(cmd, field) for field in (
            'position_rad', 'velocity_rad_s', 'speed_limit_rad_s', 'current_a', 'torque_nm',
            'feedforward_nm', 'kp', 'kd', 'current_limit_a', 'release_brake')},
        'command_valid_for_ns': stamp_ns(cmd.valid_for),
        'virtual_device': feedback.virtual_device, 'feedback_type': feedback.feedback_type,
        'motor_error': feedback.motor_error if feedback.error_valid else None,
        'safety_state': feedback.safety_state, 'brake_valid': feedback.brake_valid,
        'brake_released': feedback.brake_released if feedback.brake_valid else None,
        'rx_mono_ns': feedback.rx_mono_ns or None,
        'rx_kernel_realtime_ns': feedback.rx_kernel_realtime_ns if feedback.kernel_rx_valid else None,
        'rx_userspace_mono_ns': feedback.rx_userspace_mono_ns or None,
    }
    row.update(measurement_fields(feedback))
    return row


def raw_row(frame, bag_ns=None):
    return {
        'bag_record_ns': bag_ns, 'ros_publish_ns': stamp_ns(frame.header.stamp),
        'rx_mono_ns': frame.host_mono_ns or None,
        'kernel_realtime_ns': frame.kernel_realtime_ns if frame.kernel_realtime_valid else None,
        'can_id': f'0x{frame.id:X}', 'length': frame.length,
        'data_hex': bytes(frame.data[:frame.length]).hex(' ').upper(),
        **{field: getattr(frame, field) for field in ('fd', 'brs', 'esi', 'extended', 'rtr', 'error', 'origin')},
    }


def state_row(state, bag_ns=None):
    return {
        'bag_record_ns': bag_ns, 'ros_publish_ns': stamp_ns(state.header.stamp),
        'rx_mono_ns': state.rx_mono_ns or None, 'motor_id': state.motor_id,
        'connected': state.connected, 'safety_state': state.safety_state, 'feedback_type': state.feedback_type,
        'motor_error': state.motor_error if state.error_valid else None,
        **measurement_fields(state),
    }


def export(bag, output, storage='sqlite3', topic='/encos/control_sample'):
    from encos_msgs.msg import ControlSample, MotorState, RawFrame
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions

    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag), storage_id=storage), ConverterOptions('', ''))
    message, row = {'/encos/control_sample': (ControlSample, sample_row), '/encos/raw_rx': (RawFrame, raw_row),
                    '/encos/motor_state': (MotorState, state_row)}[topic]
    if not any(item.name == topic and item.type == f'encos_msgs/msg/{message.__name__}'
               for item in reader.get_all_topics_and_types()):
        raise ValueError(f'bag does not contain the expected message type on {topic}')
    reader.set_filter(StorageFilter(topics=[topic]))
    count = 0
    with output.open('x', newline='') as output_file:
        writer = csv.DictWriter(output_file, fieldnames=list(row(message())))
        writer.writeheader()
        while reader.has_next():
            _, payload, stamp = reader.read_next()
            writer.writerow(row(deserialize_message(payload, message), stamp))
            count += 1
    if count == 0:
        raise ValueError(f'bag contains no samples for {topic}')
    return count


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('csv', type=Path)
    parser.add_argument('--storage', default='sqlite3', choices=['sqlite3', 'mcap'])
    parser.add_argument('--topic', default='/encos/control_sample',
                        choices=['/encos/raw_rx', '/encos/motor_state', '/encos/control_sample'])
    args = parser.parse_args()
    print(f'exported {export(args.bag, args.csv, args.storage, args.topic)} samples from {args.topic}')
