#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNTIME_DIR="$SCRIPT_DIR/runtime"
PRIVATE_HOME_DIR="$SCRIPT_DIR/home"
DISPLAY_VALUE="${DISPLAY:-:0}"
XAUTHORITY_VALUE="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"

if [[ ! -x "$RUNTIME_DIR/NX/bin/nxplayer" ]]; then
  echo "NoMachine 9.8.2 runtime is incomplete: $RUNTIME_DIR/NX/bin/nxplayer" >&2
  exit 1
fi

mkdir -p "$PRIVATE_HOME_DIR/.nx/cache" "$PRIVATE_HOME_DIR/.nx/config"

exec bwrap \
  --ro-bind / / \
  --dev-bind /dev /dev \
  --proc /proc \
  --tmpfs /tmp \
  --ro-bind /tmp/.X11-unix /tmp/.X11-unix \
  --ro-bind "$RUNTIME_DIR/NX" /usr/NX \
  --bind "$PRIVATE_HOME_DIR" "$PRIVATE_HOME_DIR" \
  --setenv HOME "$PRIVATE_HOME_DIR" \
  --setenv NX_SYSTEM /usr/NX \
  --setenv DISPLAY "$DISPLAY_VALUE" \
  --setenv XAUTHORITY "$XAUTHORITY_VALUE" \
  /usr/NX/bin/nxplayer "$@"
