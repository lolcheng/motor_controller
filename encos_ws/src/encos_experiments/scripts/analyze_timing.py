#!/usr/bin/env python3
"""Summarize host receive intervals or historical ENCOS control timing."""
import argparse
import csv
import json
import math


def truth(value):
    return value is True or str(value).lower() in ('true', '1')


def percentiles(values):
    data = sorted(values)
    return {label: data[math.ceil(ratio * len(data)) - 1] if data else None
            for label, ratio in [('p50', .5), ('p95', .95), ('p99', .99), ('max', 1.)]}


def summarize(rows, hz):
    successful_tx = [row for row in rows if int(row['send_status']) == 0]
    replies = [row for row in successful_tx if truth(row['matched'])]
    expected = sum(truth(row['requested_reply']) for row in successful_tx)
    first_last = ([int(r['tx_call_before_mono_ns']) for r in successful_tx[::max(1, len(successful_tx)-1)]]
                  if successful_tx else [])
    span = (first_last[-1] - first_last[0]) / 1e9 if len(first_last) > 1 else 0
    rtts = [(int(r['rx_mono_ns']) - int(r['tx_call_before_mono_ns'])) / 1000 for r in replies]
    publish = [(int(r['ros_publish_mono_ns']) - int(r['rx_mono_ns'])) / 1000 for r in replies]
    topic_rows = [row for row in successful_tx if row['ros_generation_ns'] not in ('', None)]
    to_send = [(int(r['tx_call_before_mono_ns']) - int(r['command_generated_mono_ns'])) / 1000 for r in topic_rows]
    jitter = [abs(int(r['previous_cycle_interval_ns']) / 1000 - 1e6 / hz)
              for r in successful_tx if int(r['previous_cycle_interval_ns']) > 0]
    feedback_hz = 0
    if len(replies) > 1:
        rx_span = (int(replies[-1]['rx_mono_ns']) - int(replies[0]['rx_mono_ns'])) / 1e9
        feedback_hz = (len(replies) - 1) / rx_span if rx_span > 0 else 0
    sequences = [int(row['sequence']) for row in rows]
    return {
        'target_hz': hz, 'tx_span_s': span, 'csv_samples': len(rows), 'tx': len(successful_tx),
        'tx_hz': (len(successful_tx) - 1) / span if span > 0 else 0,
        'feedback_hz': feedback_hz, 'matched': len(replies), 'expected_replies': expected,
        'held_cycles': len(topic_rows) - len({int(row['command_received_mono_ns']) for row in topic_rows}),
        'loss_pct': ((expected - len(replies)) / expected * 100) if expected else None,
        'timeouts': sum(int(row['transaction_state']) == 5 for row in rows),
        'skipped_during_samples': int(rows[-1]['skipped_cycles']) - int(rows[0]['skipped_cycles']) if rows else 0,
        'sequence_gaps': max(sequences) - min(sequences) + 1 - len(set(sequences)) if sequences else 0,
        'rtt_us': percentiles(rtts), 'source_to_tx_us': percentiles(to_send),
        'rx_to_publish_us': percentiles(publish), 'jitter_us': percentiles(jitter),
    }


def summarize_receive(rows, hz, can_id):
    timestamps = [int(row['rx_mono_ns']) for row in rows
                  if row['rx_mono_ns'] and (can_id is None or int(row['can_id'], 0) == can_id)]
    gaps = [(later - earlier) / 1000 for earlier, later in zip(timestamps, timestamps[1:])]
    if any(gap < 0 for gap in gaps):
        raise ValueError('host receive timestamps must be monotonic within a recording')
    span = (timestamps[-1] - timestamps[0]) / 1e9 if gaps else 0
    return {
        'target_hz': hz, 'can_id': can_id, 'rx_samples': len(timestamps), 'rx_span_s': span,
        'rx_hz': len(gaps) / span if span > 0 else None,
        'host_rx_gap_us': percentiles(gaps),
        'host_rx_jitter_us': percentiles([abs(gap - 1e6 / hz) for gap in gaps]),
    }


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv')
    parser.add_argument('--hz', required=True, type=int)
    parser.add_argument('--rx', action='store_true', help='analyze a raw receive CSV from the USB-CAN node')
    parser.add_argument('--can-id', type=lambda value: int(value, 0), help='filter receive analysis by CAN ID')
    args = parser.parse_args()
    if args.hz <= 0:
        parser.error('--hz must be positive')
    if args.can_id is not None and not args.rx:
        parser.error('--can-id requires --rx')
    with open(args.csv, newline='') as source:
        rows = list(csv.DictReader(source))
    result = summarize_receive(rows, args.hz, args.can_id) if args.rx else summarize(rows, args.hz)
    print(json.dumps(result, indent=2))
