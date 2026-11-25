#!/usr/bin/env bash
set -euo pipefail
if [[ $# != 3 ]]; then
  echo 'usage: run_vcan_benchmark.sh TOOL_DIRECTORY NEW_OUTPUT_DIRECTORY VCAN_INTERFACE' >&2
  exit 2
fi
binaries=$(realpath -- "$1")
output=$2
iface=$3
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
"$script_dir/setup_vcan.sh" check "$iface"
mkdir -p -- "$(dirname -- "$output")"
mkdir -- "$output"
peer=
cleanup() {
  if [[ -n $peer ]]; then
    kill -TERM "$peer" 2>/dev/null || true
    wait "$peer" || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM

failed=0
for format in classic fd; do
  for hz in 10 50 100 200 400 500 1000; do
    case "$hz" in
      400) duration=61 ;;
      500|1000) duration=12 ;;
      *) duration=3 ;;
    esac
    prefix="$output/${format}_${hz}hz"
    "$binaries/encos_virtual_motor" "$iface" "$format" > "${prefix}_motor.log" 2>&1 &
    peer=$!
    ready=0
    for _ in {1..100}; do
      if rg -q '^ready ' "${prefix}_motor.log"; then ready=1; break; fi
      if ! kill -0 "$peer" 2>/dev/null; then break; fi
      sleep 0.02
    done
    if (( ready )); then
      if ! "$binaries/encos_latency_benchmark" "$iface" "$format" "$hz" "$duration" "${prefix}.csv" \
          2>&1 | tee "${prefix}_summary.txt"; then
        failed=1
      fi
    else
      echo 'emulator failed to start' | tee "${prefix}_summary.txt"
      failed=1
    fi
    if kill -TERM "$peer" 2>/dev/null; then
      if ! wait "$peer"; then failed=1; fi
    else
      wait "$peer" || true
      failed=1
    fi
    peer=
  done
done
exit "$failed"
