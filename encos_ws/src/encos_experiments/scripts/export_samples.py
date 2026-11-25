#!/usr/bin/env python3
"""Export actual ControlSample timestamps and field validity from a rosbag2 bag."""
import argparse
import csv
from pathlib import Path


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


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
    for index, field in enumerate(('position_rad', 'velocity_rad_s', 'phase_current_a', 'estimated_torque_nm',
                                   'motor_temperature_c', 'mos_temperature_c')):
        row[field] = getattr(feedback, field) if feedback.valid[index] else None
        row[f'{field}_valid'] = feedback.valid[index]
        row[f'{field}_source'] = feedback.source[index]
        row[f'{field}_mono_ns'] = feedback.measurement_mono_ns[index] if feedback.valid[index] else None
    return row


def export(bag, output, storage='sqlite3'):
    from encos_msgs.msg import ControlSample
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions

    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag), storage_id=storage), ConverterOptions('', ''))
    topic = '/encos/control_sample'
    if not any(item.name == topic and item.type == 'encos_msgs/msg/ControlSample'
               for item in reader.get_all_topics_and_types()):
        raise ValueError('bag does not contain the typed control sample topic')
    reader.set_filter(StorageFilter(topics=[topic]))
    count = 0
    with output.open('x', newline='') as output_file:
        writer = csv.DictWriter(output_file, fieldnames=list(sample_row(ControlSample())))
        writer.writeheader()
        while reader.has_next():
            _, payload, stamp = reader.read_next()
            writer.writerow(sample_row(deserialize_message(payload, ControlSample), stamp))
            count += 1
    if count == 0:
        raise ValueError('bag contains no control samples')
    return count


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('csv', type=Path)
    parser.add_argument('--storage', default='sqlite3', choices=['sqlite3', 'mcap'])
    args = parser.parse_args()
    print(f'exported {export(args.bag, args.csv, args.storage)} samples')
