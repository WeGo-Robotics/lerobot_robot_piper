#!/usr/bin/env python3
"""Show CAN adapter serials and their current interface names.

Used when (re)generating the serial-pinned udev rules
(install_can_persist.sh). If an arm is rewired to a different adapter the
rules must change too — this shows which adapter is which.

The serial alone cannot tell you which ARM it is: both arms usually run the
same firmware, so the only reliable check is to move an arm and watch:

    python3 scripts/list_can_adapters.py --watch

Move one arm by hand; the interface it is wired to prints a line. Verify with
this BEFORE trusting freshly written rules.

--watch talks to the arms directly via piper_sdk, so close piper-ui /
piper-teleop / any teleoperation process first — CAN must have one owner.
Interfaces must already be up (run scripts/can_restore.sh or can_activate.sh).
"""

import subprocess
import sys
from pathlib import Path

USB = Path("/sys/bus/usb/devices")
GS_USB_IDS = ("1d50", "606f")  # candleLight / gs_usb
MOVE_THRESHOLD = 5000          # raw units (0.001 deg) — easily exceeded by hand


def _attr(node: Path, name: str) -> str:
    try:
        return (node / name).read_text().strip()
    except OSError:
        return ""


def _can_ifaces() -> list[str]:
    out = subprocess.run(["ip", "-br", "link", "show", "type", "can"],
                         capture_output=True, text=True).stdout
    return [line.split()[0] for line in out.splitlines() if line.strip()]


def _read_joints_raw(piper) -> list[int] | None:
    """Raw joint positions; reads both leader-broadcast and encoder feedback.

    Same logic as arm_setup_ui.ArmInfo.read_joints_raw — works regardless of
    leader/follower mode.
    """
    ctrl_vals = state_vals = None
    try:
        j = piper.GetArmJointCtrl().joint_ctrl
        ctrl_vals = [j.joint_1, j.joint_2, j.joint_3, j.joint_4, j.joint_5, j.joint_6]
    except Exception:
        pass
    try:
        j = piper.GetArmJointMsgs().joint_state
        state_vals = [j.joint_1, j.joint_2, j.joint_3, j.joint_4, j.joint_5, j.joint_6]
    except Exception:
        pass
    if ctrl_vals and any(v != 0 for v in ctrl_vals):
        return ctrl_vals
    return state_vals


def watch() -> int:
    import time

    from piper_sdk import C_PiperInterface_V2

    ifaces = _can_ifaces()
    if not ifaces:
        print("No CAN interfaces up. Run scripts/can_restore.sh (or can_activate.sh) first.")
        return 1

    arms = {}
    for i in ifaces:
        try:
            piper = C_PiperInterface_V2(i, judge_flag=False, can_auto_init=False)
            piper.CreateCanBus(i)
            piper.ConnectPort(piper_init=False, start_thread=True)
            arms[i] = piper
        except Exception as e:
            print(f"  {i}: connect failed ({e}) — skipping")

    if not arms:
        print("Could not connect to any interface.")
        return 1

    time.sleep(0.5)  # let the SDK reader threads fill in first frames
    base = {i: _read_joints_raw(p) for i, p in arms.items()}
    print(f"Move one arm by hand. Watching: {', '.join(arms)}   (Ctrl-C to quit)",
          flush=True)
    try:
        while True:
            time.sleep(0.2)
            for i, p in arms.items():
                now = _read_joints_raw(p)
                if not now or not base[i]:
                    base[i] = now
                    continue
                delta = max(abs(a - b) for a, b in zip(now, base[i]))
                if delta > MOVE_THRESHOLD:
                    print(f"  > {i} moved (d{delta})", flush=True)
                    base[i] = now
    except KeyboardInterrupt:
        return 0
    finally:
        for p in arms.values():
            try:
                p.DisconnectPort()
            except Exception:
                pass


def main() -> int:
    if "--watch" in sys.argv:
        return watch()

    # interface -> USB device node ("3-11.1:1.0/net/can0" -> "3-11.1")
    iface_of: dict[str, str] = {}
    for net in Path("/sys/class/net").iterdir():
        dev = net / "device"
        if not dev.exists():
            continue
        for part in dev.resolve().parts:
            if part.count("-") == 1 and part[0].isdigit() and ":" not in part:
                iface_of[part] = net.name

    rows = []
    for node in sorted(USB.iterdir()):
        if (_attr(node, "idVendor"), _attr(node, "idProduct")) != GS_USB_IDS:
            continue
        rows.append((node.name, _attr(node, "serial"), iface_of.get(node.name, "-")))

    if not rows:
        print(f"No CAN adapters ({GS_USB_IDS[0]}:{GS_USB_IDS[1]}) found.")
        return 1

    print(f"{'USB':<10} {'serial':<26} {'iface':<16} state")
    for usb, serial, iface in rows:
        state = ""
        if iface != "-":
            out = subprocess.run(["ip", "-br", "link", "show", iface],
                                 capture_output=True, text=True).stdout.split()
            state = out[1] if len(out) > 1 else ""
        print(f"{usb:<10} {serial:<26} {iface:<16} {state}")

    print("\nWhich serial is which ARM cannot be read from this table — "
          "run with --watch and move an arm to find out.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
