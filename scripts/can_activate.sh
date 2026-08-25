#!/bin/bash
# can_activate.sh — activate a single Piper CAN interface.
#
# Based on piper_sdk's can_activate.sh (agilexrobotics/piper_sdk), simplified
# and made idempotent. For multi-arm setups, run `piper-setup` once and use
# can_restore.sh instead.
#
# Usage:
#   bash scripts/can_activate.sh [NAME] [BITRATE] [BUS_INFO]
#
#   NAME      target interface name         (default: can0)
#   BITRATE   CAN bitrate in bit/s          (default: 1000000)
#   BUS_INFO  USB address, e.g. "1-2:1.0"   (required when >1 CAN module
#             is plugged in; find it with:  sudo ethtool -i can0 | grep bus-info)

set -u

TARGET_NAME="${1:-can0}"
BITRATE="${2:-1000000}"
BUS_INFO="${3:-}"

SUDO=""
[ "$(id -u)" -ne 0 ] && SUDO="sudo"

for tool in ip ethtool; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        echo "Error: '$tool' not found. Install it with:"
        echo "  sudo apt update && sudo apt install iproute2 ethtool can-utils"
        exit 1
    fi
done

$SUDO modprobe gs_usb 2>/dev/null || true

mapfile -t IFACES < <(ip -br link show type can | awk '{print $1}')
COUNT=${#IFACES[@]}

if [ "$COUNT" -eq 0 ]; then
    echo "Error: no CAN interface detected. Is the CAN-USB adapter plugged in?"
    exit 1
fi

# Resolve which physical interface to configure.
if [ -n "$BUS_INFO" ]; then
    IFACE=""
    for i in "${IFACES[@]}"; do
        bi=$($SUDO ethtool -i "$i" | awk '/bus-info/{print $2}')
        [ "$bi" = "$BUS_INFO" ] && IFACE="$i" && break
    done
    if [ -z "$IFACE" ]; then
        echo "Error: no CAN interface with bus-info '$BUS_INFO'. Detected:"
        for i in "${IFACES[@]}"; do
            echo "  $i  $($SUDO ethtool -i "$i" | awk '/bus-info/{print $2}')"
        done
        exit 1
    fi
elif [ "$COUNT" -eq 1 ]; then
    IFACE="${IFACES[0]}"
else
    echo "Error: $COUNT CAN modules detected but no BUS_INFO given."
    echo "Identify each module's USB address and re-run:"
    for i in "${IFACES[@]}"; do
        echo "  $i  $($SUDO ethtool -i "$i" | awk '/bus-info/{print $2}')"
    done
    echo "Example: bash scripts/can_activate.sh can0 1000000 1-2:1.0"
    exit 1
fi

CURRENT_BITRATE=$(ip -details link show "$IFACE" | grep -oP 'bitrate \K\d+' || true)
IS_UP=$(ip link show "$IFACE" | grep -q "state UP" && echo yes || echo no)

if [ "$IS_UP" = "yes" ] && [ "$CURRENT_BITRATE" = "$BITRATE" ] && [ "$IFACE" = "$TARGET_NAME" ]; then
    echo "$TARGET_NAME already up at $BITRATE bit/s. Nothing to do."
    exit 0
fi

$SUDO ip link set "$IFACE" down
if [ "$IFACE" != "$TARGET_NAME" ]; then
    $SUDO ip link set "$IFACE" name "$TARGET_NAME"
    IFACE="$TARGET_NAME"
fi
$SUDO ip link set "$IFACE" type can bitrate "$BITRATE"
$SUDO ip link set "$IFACE" up

echo "$IFACE up at $BITRATE bit/s."
