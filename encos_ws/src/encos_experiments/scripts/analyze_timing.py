#!/usr/bin/env python3
"""Summarize real monotonic timings in a P2 control sample CSV."""
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


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv')
    parser.add_argument('--hz', required=True, type=int)
    args = parser.parse_args()
    if args.hz <= 0:
        parser.error('--hz must be positive')
    with open(args.csv, newline='') as source:
        print(json.dumps(summarize(list(csv.DictReader(source)), args.hz), indent=2))
