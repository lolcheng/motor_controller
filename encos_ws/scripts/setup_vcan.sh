#!/usr/bin/env bash
set -euo pipefail
action=${1:-check}
iface=${2:-vcan0}
[[ $iface =~ ^[a-zA-Z0-9_.-]{1,15}$ && $iface != -* ]] || { echo 'invalid interface name' >&2; exit 2; }

show() { ip -details -json link show dev "$iface" 2>/dev/null; }
verify() {
  local require_up=$1
  python3 -c '
import json, sys
link, = json.load(sys.stdin)
if link.get("linkinfo", {}).get("info_kind") != "vcan":
    raise SystemExit("interface is not Linux vcan")
if sys.argv[1] == "up" and ("UP" not in link.get("flags", []) or link.get("mtu", 0) < 72):
    raise SystemExit("vcan must be UP with MTU >= 72")
' "$require_up"
}
case "$action" in
  check)
    if ! state=$(show); then echo "vcan interface unavailable: $iface" >&2; exit 77; fi
    verify up <<< "$state"
    ip -details link show dev "$iface"
    ;;
  ensure)
    if ! state=$(show); then
      ip link add dev "$iface" type vcan
      state=$(show)
    fi
    verify kind <<< "$state"
    if ! verify up <<< "$state" 2>/dev/null; then
      ip link set dev "$iface" down
      ip link set dev "$iface" mtu 72
      ip link set dev "$iface" up
    fi
    "$0" check "$iface"
    ;;
  clean)
    if state=$(show); then
      verify kind <<< "$state"
      ip link del dev "$iface"
    fi
    ;;
  *) echo 'usage: setup_vcan.sh [check|ensure|clean] [vcan0]' >&2; exit 2 ;;
esac
