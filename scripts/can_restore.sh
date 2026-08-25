#!/bin/bash
# can_restore.sh — bring up Piper CAN interfaces from piper_config.json.
#
# `piper-setup` (the multi-arm wizard) saves each arm's USB address (bus-info)
# and canonical name (can_leader1, can_follower1, ...) to ~/piper_config.json.
# For every configured arm this script finds the interface currently on that
# USB port, renames it if needed, sets the bitrate, and brings it up.
# Safe to run repeatedly; already-correct interfaces are skipped.
#
# When install_can_persist.sh has installed serial-pinned udev rules, the
# kernel already names interfaces correctly at plug time and this script only
# sets the bitrate and brings them up (rename becomes a no-op). Without those
# rules it still works, but the mapping then follows the USB *port*, not the
# adapter — swap ports and the names swap arms.
#
# Arms whose USB port is empty (unplugged) are skipped, not errors — the udev
# hotplug trigger re-runs this script when they appear.
#
# Usage:
#   bash scripts/can_restore.sh                 # config: ~/piper_config.json
#   PIPER_CONFIG=/path/to/piper_config.json bash scripts/can_restore.sh
#   PIPER_BITRATE=1000000 bash scripts/can_restore.sh

set -u

CONFIG="${PIPER_CONFIG:-$HOME/piper_config.json}"
BITRATE="${PIPER_BITRATE:-1000000}"

SUDO=""
[ "$(id -u)" -ne 0 ] && SUDO="sudo"

if [ ! -f "$CONFIG" ]; then
    echo "Error: config not found: $CONFIG"
    echo "Run 'piper-setup' first to assign arm names, or use can_activate.sh for a single arm."
    exit 1
fi

for tool in ip ethtool python3; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        echo "Error: '$tool' not found."
        exit 1
    fi
done

$SUDO modprobe gs_usb 2>/dev/null || true

# "bus_info can_name" per line
MAPPING=$(python3 - "$CONFIG" <<'EOF'
import json, sys
with open(sys.argv[1]) as f:
    data = json.load(f)
for arm in data.get("arms", []):
    bus, name = arm.get("bus_info"), arm.get("can_name")
    if bus and name:
        print(bus, name)
EOF
)

if [ -z "$MAPPING" ]; then
    echo "Error: no arm entries in $CONFIG"
    exit 1
fi

# Wait briefly for interfaces to register (matters right after boot/hotplug).
for _ in 1 2 3 4 5; do
    ip -br link show type can | grep -q . && break
    sleep 1
done

restore_one() {
    local bus_info="$1" target="$2" iface="" bi
    # Prefer an interface already carrying the target name (udev rules did the
    # rename); fall back to locating by USB address.
    if ip link show "$target" >/dev/null 2>&1; then
        iface="$target"
    else
        for i in $(ip -br link show type can | awk '{print $1}'); do
            bi=$($SUDO ethtool -i "$i" | awk '/bus-info/{print $2}')
            [ "$bi" = "$bus_info" ] && iface="$i" && break
        done
    fi
    if [ -z "$iface" ]; then
        echo "  skip: $target not present (USB $bus_info empty)"
        return 0
    fi

    local current_bitrate is_up
    current_bitrate=$(ip -details link show "$iface" | grep -oP 'bitrate \K\d+' || true)
    is_up=$(ip link show "$iface" | grep -q "state UP" && echo yes || echo no)
    if [ "$iface" = "$target" ] && [ "$is_up" = "yes" ] && [ "$current_bitrate" = "$BITRATE" ]; then
        echo "  ok:   $target already up at $BITRATE"
        return 0
    fi

    $SUDO ip link set "$iface" down
    if [ "$iface" != "$target" ]; then
        $SUDO ip link set "$iface" name "$target"
        iface="$target"
    fi
    $SUDO ip link set "$iface" type can bitrate "$BITRATE"
    $SUDO ip link set "$iface" up
    echo "  done: $bus_info -> $target ($BITRATE bit/s)"
    return 0
}

echo "Restoring CAN interfaces from $CONFIG"
while read -r bus_info target; do
    restore_one "$bus_info" "$target"
done <<< "$MAPPING"

exit 0
