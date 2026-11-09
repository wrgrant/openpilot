#!/usr/bin/env bash
# Tailscale on the comma, so the Studio can SSH to it over William's tailnet when it's on a phone hotspot.
#
#   tools/odyssey_tailscale.sh install   download the static arm64 build into /data/tailscale (survives openpilot
#                                        and AGNOS updates), start it, and print the login link to approve once
#   tools/odyssey_tailscale.sh start     start tailscaled if installed; launch_chffrplus.sh calls this at boot
#   tools/odyssey_tailscale.sh stop
#   tools/odyssey_tailscale.sh status
#
# tailscaled runs as the comma user in userspace networking mode: it doesn't touch the device's routes or DNS, and
# connections to the device's tailnet address are handed to its local services (sshd on 22).
set -euo pipefail

TS_DIR=/data/tailscale
SOCK=$TS_DIR/tailscaled.sock
TS="$TS_DIR/tailscale --socket=$SOCK"

start() {
  [ -x "$TS_DIR/tailscaled" ] || exit 0
  pgrep -x tailscaled >/dev/null && exit 0
  mkdir -p "$TS_DIR/state"
  nohup nice -n 10 "$TS_DIR/tailscaled" --tun=userspace-networking --statedir="$TS_DIR/state" --socket="$SOCK" \
    --port=41641 > "$TS_DIR/tailscaled.log" 2>&1 &
}

case "${1:-}" in
  install)
    mkdir -p "$TS_DIR"
    ver=$(curl -fsSL "https://pkgs.tailscale.com/stable/?mode=json" | python3 -c "import json,sys; print(json.load(sys.stdin)['TarballsVersion'])")
    tmp=$(mktemp -d)
    curl -fsSL "https://pkgs.tailscale.com/stable/tailscale_${ver}_arm64.tgz" | tar -xz -C "$tmp"
    cp "$tmp"/tailscale_*_arm64/tailscale "$tmp"/tailscale_*_arm64/tailscaled "$TS_DIR/"
    rm -rf "$tmp"
    echo "installed tailscale $ver in $TS_DIR"
    start
    sleep 3
    $TS up --hostname=odyssey-comma --timeout=5s 2>&1 || true  # prints the login link the first time
    ;;
  start) start ;;
  stop) pkill -x tailscaled || true ;;
  status) $TS status ;;
  *) echo "usage: $0 install|start|stop|status" >&2; exit 2 ;;
esac
