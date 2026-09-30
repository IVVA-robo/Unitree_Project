#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INTERFACE_NAME="enxb4b024be59fe"
LOCAL_ADDRESS="192.168.123.162"
ROBOT_ADDRESS="192.168.123.164"
ROBOT_PORT="4000"
SESSION_FILE="$SCRIPT_DIR/home/.nx/cache/r1-192.168.123.164.nxs"

if [[ ! -r "$SESSION_FILE" ]]; then
  echo "NoMachine connection profile is missing: $SESSION_FILE" >&2
  exit 1
fi

if ! ip -brief address show dev "$INTERFACE_NAME" 2>/dev/null | grep -q "UP.*${LOCAL_ADDRESS}/24"; then
  echo "Unitree Ethernet is not ready on $INTERFACE_NAME ($LOCAL_ADDRESS/24)." >&2
  exit 2
fi

ROUTE_TEXT="$(ip route get "$ROBOT_ADDRESS" 2>/dev/null || true)"
if [[ "$ROUTE_TEXT" != *"dev $INTERFACE_NAME"* || "$ROUTE_TEXT" != *"src $LOCAL_ADDRESS"* ]]; then
  echo "Unsafe route to $ROBOT_ADDRESS: $ROUTE_TEXT" >&2
  exit 3
fi

if ! timeout 3 bash -c "</dev/tcp/$ROBOT_ADDRESS/$ROBOT_PORT" 2>/dev/null; then
  echo "NoMachine server is not responding at $ROBOT_ADDRESS:$ROBOT_PORT." >&2
  exit 4
fi

# Force keyboard capture while the pointer is inside the remote desktop.
# This is useful on Linux desktops where global X11 grabs are unavailable.
exec "$SCRIPT_DIR/run-nxplayer-9.7.3.sh" --activegrab --session "$SESSION_FILE"
