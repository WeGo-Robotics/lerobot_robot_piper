"""piper-doctor - diagnose a Piper + LeRobot installation.

Checks run in dependency order: if an early layer is broken, later failures
are usually just symptoms of it. Each failed check prints a prescription -
the next command to run.

  1. Python packages & LeRobot plugin registration
  2. CAN adapter hardware (USB)
  3. CAN interface state vs ~/piper_config.json
  4. Arm response (passive RX counters, then active firmware query)
  5. Boot persistence (udev rules + systemd service)

Read-only: never changes interface state, never sends arm commands.
Exit code 0 = all checks passed (warnings allowed), 1 = at least one failure.

Entry point: piper-doctor
"""

from __future__ import annotations

import json
import os
import platform
import subprocess
import sys
import time
from pathlib import Path

GS_USB_IDS = ("1d50", "606f")  # candleLight / gs_usb
DEFAULT_BITRATE = 1_000_000
CONFIG_PATH = Path(os.environ.get("PIPER_CONFIG", "")) if os.environ.get("PIPER_CONFIG") \
    else Path.home() / "piper_config.json"

OK, WARN, FAIL, SKIP = "OK", "WARN", "FAIL", "SKIP"
_COLORS = {"OK": "\033[32m", "WARN": "\033[33m", "FAIL": "\033[31m", "SKIP": "\033[90m"}
_USE_COLOR = sys.stdout.isatty() and platform.system() != "Windows" or bool(os.environ.get("FORCE_COLOR"))


def _tag(status: str) -> str:
    if _USE_COLOR:
        return f"{_COLORS[status]}{status:<4}\033[0m"
    return f"{status:<4}"


class Report:
    def __init__(self):
        self.failed = False

    def check(self, status: str, name: str, detail: str = "", fix: str = ""):
        line = f"  [{_tag(status)}] {name}"
        if detail:
            line += f" - {detail}"
        print(line)
        if fix:
            print(f"         fix: {fix}")
        if status == FAIL:
            self.failed = True

    def section(self, n: int, total: int, title: str):
        print(f"\n[{n}/{total}] {title}")


def _run(cmd: list[str]) -> tuple[int, str]:
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        return p.returncode, p.stdout.strip()
    except (OSError, subprocess.TimeoutExpired) as e:
        return 1, str(e)


def _can_ifaces() -> list[tuple[str, str]]:
    """[(iface, state), ...] from `ip -br link show type can`."""
    rc, out = _run(["ip", "-br", "link", "show", "type", "can"])
    if rc != 0:
        return []
    rows = []
    for line in out.splitlines():
        parts = line.split()
        if parts:
            rows.append((parts[0], parts[1] if len(parts) > 1 else "?"))
    return rows


def _iface_bitrate(iface: str) -> int | None:
    rc, out = _run(["ip", "-details", "link", "show", iface])
    if rc != 0:
        return None
    for tok in ("bitrate ",):
        idx = out.find(tok)
        if idx >= 0:
            try:
                return int(out[idx + len(tok):].split()[0])
            except (ValueError, IndexError):
                pass
    return None


def _rx_packets(iface: str) -> int | None:
    try:
        return int(Path(f"/sys/class/net/{iface}/statistics/rx_packets").read_text())
    except (OSError, ValueError):
        return None


# ── 1. packages & plugin registration ───────────────────────────────────────

def check_packages(r: Report) -> bool:
    """Returns True if the plugin itself imported (later checks need it)."""
    v = sys.version_info
    if v >= (3, 10):
        r.check(OK, f"Python {v.major}.{v.minor}.{v.micro}")
    else:
        r.check(FAIL, f"Python {v.major}.{v.minor}", "plugin requires >= 3.10",
                "install Python 3.10+ (e.g. via conda/pyenv) and reinstall")
        return False

    try:
        import piper_sdk  # noqa: F401
        r.check(OK, "piper_sdk", getattr(piper_sdk, "__version__", ""))
    except ImportError:
        r.check(FAIL, "piper_sdk not importable", "",
                "pip install piper_sdk  (or reinstall the plugin: pip install -e .)")

    try:
        import lerobot
        r.check(OK, "lerobot", getattr(lerobot, "__version__", ""))
    except ImportError:
        r.check(FAIL, "lerobot not importable", "",
                "pip install 'lerobot>=0.3.0'  (or reinstall the plugin: pip install -e .)")
        return False

    try:
        from lerobot.robots.config import RobotConfig
        from lerobot.teleoperators.config import TeleoperatorConfig
    except ImportError as e:
        r.check(WARN, "could not inspect LeRobot registry", str(e))
        return True

    def registered(base, name) -> bool:
        try:
            if hasattr(base, "get_known_choices"):
                return name in base.get_known_choices()
            return base.get_choice_class(name) is not None
        except Exception:
            return False

    # Mirror the CLI first: newer lerobot auto-imports installed distributions
    # named lerobot_robot_* / lerobot_teleoperator_* / ... at startup. If that
    # path finds us, `--robot.type=piper_follower` will work.
    discovery = None
    try:
        from lerobot.utils.import_utils import register_third_party_plugins
        register_third_party_plugins()
        discovery = True
    except ImportError:
        discovery = False   # older lerobot: registration happens on manual import
    except Exception as e:
        r.check(WARN, "plugin auto-discovery raised", str(e))

    targets = ((RobotConfig, "piper_follower"), (TeleoperatorConfig, "piper_leader"))
    missing = [(b, n) for b, n in targets if not registered(b, n)]

    if not missing:
        via = "auto-discovered by lerobot" if discovery else "registered"
        for _, n in targets:
            r.check(OK, f"'{n}' {via}")
        return True

    # Not registered yet - import the plugin package directly to tell a broken
    # install apart from broken discovery.
    try:
        import lerobot_robot_piper  # noqa: F401
    except Exception as e:
        r.check(FAIL, "lerobot_robot_piper import failed", str(e),
                "pip install -e .  (from the plugin repo)")
        return False

    still_missing = [n for b, n in targets if not registered(b, n)]
    if still_missing:
        for n in still_missing:
            r.check(FAIL, f"'{n}' NOT in LeRobot registry",
                    "plugin imports but does not register the type",
                    "reinstall: pip install -e .  - and report this if it persists")
        return False

    if discovery:
        r.check(FAIL, "plugin registers on manual import but lerobot auto-discovery missed it",
                "the CLI (--robot.type=piper_follower) will NOT find it",
                "reinstall: pip install -e .  - the installed distribution must be named lerobot_robot_piper")
    else:
        for _, n in targets:
            r.check(OK, f"'{n}' registered on import")
    return True


# ── 2. adapter hardware ─────────────────────────────────────────────────────

def check_adapters(r: Report) -> int:
    usb = Path("/sys/bus/usb/devices")
    adapters = []
    if usb.exists():
        for node in usb.iterdir():
            try:
                vid = (node / "idVendor").read_text().strip()
                pid = (node / "idProduct").read_text().strip()
            except OSError:
                continue
            if (vid, pid) == GS_USB_IDS:
                try:
                    serial = (node / "serial").read_text().strip()
                except OSError:
                    serial = "(no serial)"
                adapters.append((node.name, serial))

    if adapters:
        r.check(OK, f"{len(adapters)} CAN-USB adapter(s) on USB",
                ", ".join(f"{u}:{s}" for u, s in adapters))
    else:
        r.check(FAIL, "no CAN-USB adapter (1d50:606f) on USB bus", "",
                "plug in the CAN adapter; check the USB cable")

    if not Path("/sys/module/gs_usb").exists():
        r.check(WARN, "gs_usb kernel module not loaded",
                "usually auto-loads on plug",
                "sudo modprobe gs_usb")

    ifaces = _can_ifaces()
    if ifaces:
        r.check(OK, f"{len(ifaces)} CAN network interface(s)",
                ", ".join(f"{i}({s})" for i, s in ifaces))
    elif adapters:
        r.check(FAIL, "adapter on USB but no CAN network interface", "",
                "sudo modprobe gs_usb; replug the adapter; check `dmesg | tail`")
    return len(ifaces)


# ── 3. interface state vs config ────────────────────────────────────────────

def check_interfaces(r: Report) -> list[str]:
    """Returns the list of UP interfaces worth probing for arm response."""
    ifaces = dict(_can_ifaces())

    if not CONFIG_PATH.exists():
        if len(ifaces) <= 1:
            r.check(WARN, f"no {CONFIG_PATH}",
                    "fine for a single arm on can0",
                    "for multi-arm setups run: piper-setup")
        else:
            r.check(FAIL, f"no {CONFIG_PATH} but {len(ifaces)} CAN interfaces",
                    "arm names are enumeration order - they can swap",
                    "run: piper-setup  (then: sudo bash scripts/install_can_persist.sh)")
        up = []
        for name, state in ifaces.items():
            ok = state == "UP" and _iface_bitrate(name) == DEFAULT_BITRATE
            if ok:
                r.check(OK, f"{name} UP @ {DEFAULT_BITRATE}")
                up.append(name)
            else:
                r.check(FAIL, f"{name} {state}, bitrate {_iface_bitrate(name)}", "",
                        f"bash scripts/can_activate.sh {name} {DEFAULT_BITRATE}")
        return up

    try:
        arms = json.loads(CONFIG_PATH.read_text()).get("arms", [])
    except (OSError, json.JSONDecodeError) as e:
        r.check(FAIL, f"cannot read {CONFIG_PATH}", str(e), "re-run: piper-setup")
        return []

    r.check(OK, f"config {CONFIG_PATH}", f"{len(arms)} arm(s)")
    up = []
    for arm in arms:
        name = arm.get("can_name")
        if not name:
            continue
        if name not in ifaces:
            r.check(FAIL, f"{name} missing",
                    f"expected on USB {arm.get('bus_info')}",
                    "sudo bash scripts/can_restore.sh  (arm unplugged? adapter moved? re-run piper-setup)")
            continue
        state = ifaces[name]
        bitrate = _iface_bitrate(name)
        if state == "UP" and bitrate == DEFAULT_BITRATE:
            r.check(OK, f"{name} UP @ {bitrate}")
            up.append(name)
        else:
            r.check(FAIL, f"{name} state={state} bitrate={bitrate}", "",
                    "sudo bash scripts/can_restore.sh")
    return up


# ── 4. arm response ─────────────────────────────────────────────────────────

def check_arms(r: Report, up_ifaces: list[str]):
    if not up_ifaces:
        r.check(SKIP, "no UP interfaces to probe")
        return

    quiet = []
    for iface in up_ifaces:
        rx1 = _rx_packets(iface)
        if rx1 is None:
            r.check(WARN, f"{iface}: cannot read RX counter")
            continue
        time.sleep(0.6)
        rx2 = _rx_packets(iface) or rx1
        if rx2 > rx1:
            r.check(OK, f"{iface}: arm broadcasting", f"+{rx2 - rx1} frames/0.6s")
        else:
            quiet.append(iface)

    # Quiet bus: a standby arm may not broadcast - try an active firmware query.
    for iface in quiet:
        fw = _query_firmware(iface)
        if fw:
            r.check(OK, f"{iface}: arm responds", f"firmware {fw}")
        else:
            r.check(FAIL, f"{iface}: no CAN traffic and no firmware reply", "",
                    "check arm POWER and the CAN cable to this adapter; "
                    "release e-stop if pressed")


def _query_firmware(iface: str) -> str | None:
    try:
        from piper_sdk import C_PiperInterface_V2
        piper = C_PiperInterface_V2(iface, judge_flag=False, can_auto_init=False)
        piper.CreateCanBus(iface)
        piper.ConnectPort(piper_init=False, start_thread=True)
        try:
            piper.SearchPiperFirmwareVersion()
        except Exception:
            pass
        for _ in range(15):
            time.sleep(0.2)
            fw = piper.GetPiperFirmwareVersion()
            if isinstance(fw, str):
                return fw
        return None
    except Exception:
        return None
    finally:
        try:
            piper.DisconnectPort()
        except Exception:
            pass


# ── 5. boot persistence ─────────────────────────────────────────────────────

def check_persistence(r: Report):
    rules = Path("/etc/udev/rules.d/99-piper-can.rules")
    if rules.exists():
        r.check(OK, "udev rules installed", str(rules))
    else:
        r.check(WARN, "no serial-pinned udev rules",
                "names follow USB enumeration and can swap between arms",
                "sudo bash scripts/install_can_persist.sh")

    rc, out = _run(["systemctl", "is-enabled", "piper-can-up.service"])
    if rc == 0 and out == "enabled":
        r.check(OK, "piper-can-up.service enabled")
    else:
        r.check(WARN, "piper-can-up.service not enabled",
                "CAN will not come up on boot",
                "sudo bash scripts/install_can_persist.sh")


# ── main ────────────────────────────────────────────────────────────────────

def main() -> int:
    r = Report()
    total = 5
    print("piper-doctor - Piper + LeRobot installation check")

    r.section(1, total, "Python packages & plugin registration")
    plugin_ok = check_packages(r)

    if platform.system() != "Linux":
        print(f"\n[2-{total}/{total}] hardware checks skipped - not Linux "
              f"({platform.system()}); CAN requires Linux SocketCAN")
        print()
        return 1 if r.failed else 0

    r.section(2, total, "CAN adapter hardware")
    n_ifaces = check_adapters(r)

    r.section(3, total, "CAN interface state")
    up = check_interfaces(r) if n_ifaces else []
    if not n_ifaces:
        r.check(SKIP, "no interfaces to check")

    r.section(4, total, "Arm response")
    if plugin_ok:
        check_arms(r, up)
    else:
        r.check(SKIP, "packages broken - fix section 1 first")

    r.section(5, total, "Boot persistence")
    check_persistence(r)

    print()
    if r.failed:
        print("RESULT: problems found - apply the fixes above, top to bottom, then re-run.")
        return 1
    print("RESULT: all good. Try:  piper-ui   or   piper-teleop")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
