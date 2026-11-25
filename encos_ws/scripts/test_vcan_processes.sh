#!/usr/bin/env bash
set -euo pipefail
if [[ $# != 3 ]]; then
  echo 'usage: test_vcan_processes.sh TOOL_DIRECTORY LOG_DIRECTORY VCAN_INTERFACE' >&2
  exit 2
fi
bin=$(realpath -- "$1")
logs=$2
iface=$3
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
"$script_dir/setup_vcan.sh" check "$iface"
mkdir -p -- "$logs"
peer=
monitor=
finish() {
  for pid in "$peer" "$monitor"; do
    if [[ -n $pid ]]; then
      kill -TERM "$pid" 2>/dev/null || true
      wait "$pid" || true
    fi
  done
}
trap finish EXIT
trap 'exit 130' INT TERM

start_peer() {
  local output=$1
  "$bin/encos_virtual_motor" "$iface" "$wire" --idle-beacon > "$output" 2>&1 &
  peer=$!
  for _ in {1..100}; do
    if rg -q '^ready ' "$output"; then return 0; fi
    kill -0 "$peer" 2>/dev/null || return 1
    sleep 0.02
  done
  return 1
}
for wire in classic fd; do
  "$bin/encos_bus_monitor" "$iface" "$wire" > "$logs/${wire}_monitor.log" 2>&1 &
  monitor=$!
  start_peer "$logs/${wire}_before_exit.log"
  "$bin/encos_cli" "$iface" "$wire" discover > "$logs/${wire}_discovery.log"
  "$bin/encos_cli" "$iface" "$wire" query 29 > "$logs/${wire}_uuid.log"
  "$bin/encos_cli" "$iface" "$wire" position 0.10 --arm-virtual > "$logs/${wire}_position.log"
  kill -KILL "$peer"
  if wait "$peer"; then
    echo 'peer should have exited via SIGKILL' >&2
    exit 1
  else
    status=$?
    [[ $status == 137 ]]
  fi
  peer=
  if timeout 2 "$bin/encos_cli" "$iface" "$wire" discover > "$logs/${wire}_without_peer.log" 2>&1; then
    echo 'unexpected successful query with peer gone' >&2
    exit 1
  else
    status=$?
    [[ $status == 1 ]]
  fi
  start_peer "$logs/${wire}_restarted.log"
  "$bin/encos_cli" "$iface" "$wire" query 2 > "$logs/${wire}_after_restart.log"
  rg -q '001#A00200000000' "$logs/${wire}_after_restart.log"
  kill -TERM "$peer" "$monitor"
  wait "$peer"
  wait "$monitor"
  peer=
  monitor=
  rg -q '7FF#FFFF0082' "$logs/${wire}_monitor.log"
  echo "$wire: CLI, monitor, SIGKILL, bounded timeout and fresh stationary restart PASS"
done
