import json
import logging
import os
import subprocess
import threading
import time
import tkinter as tk
from tkinter import ttk

import cv2
from PIL import Image, ImageTk
from piper_sdk import C_PiperInterface_V2

from lerobot.motors import Motor, MotorCalibration, MotorNormMode

from .motors import PiperMotorsBus
from .motors.tables import INITIALIZE_POSITION

logger = logging.getLogger(__name__)

_GEOMETRY_FILE = os.path.expanduser("~/.piper_ui_geometry.json")


def _load_geometry(key: str) -> str | None:
    try:
        with open(_GEOMETRY_FILE) as f:
            return json.load(f).get(key)
    except Exception:
        return None


def _save_geometry(key: str, geometry: str) -> None:
    data: dict = {}
    try:
        with open(_GEOMETRY_FILE) as f:
            data = json.load(f)
    except Exception:
        pass
    data[key] = geometry
    try:
        with open(_GEOMETRY_FILE, "w") as f:
            json.dump(data, f)
    except Exception:
        pass

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]

SLIDER_RANGE = {
    "joint1": (-100, 100),
    "joint2": (-100, 100),
    "joint3": (-100, 100),
    "joint4": (-100, 100),
    "joint5": (-100, 100),
    "joint6": (-100, 100),
    "gripper": (0, 100),
}

DEFAULT_MOTORS = {
    "joint1": Motor(1, "AGILEX-M", MotorNormMode.RANGE_M100_100),
    "joint2": Motor(2, "AGILEX-M", MotorNormMode.RANGE_M100_100),
    "joint3": Motor(3, "AGILEX-M", MotorNormMode.RANGE_M100_100),
    "joint4": Motor(4, "AGILEX-S", MotorNormMode.RANGE_M100_100),
    "joint5": Motor(5, "AGILEX-S", MotorNormMode.RANGE_M100_100),
    "joint6": Motor(6, "AGILEX-S", MotorNormMode.RANGE_M100_100),
    "gripper": Motor(7, "AGILEX-S", MotorNormMode.RANGE_0_100),
}

DEFAULT_CALIBRATION = {
    "joint1": MotorCalibration(1, 0, 0, -150000, 150000),
    "joint2": MotorCalibration(2, 0, 0, 0, 180000),
    "joint3": MotorCalibration(3, 0, 0, -170000, 0),
    "joint4": MotorCalibration(4, 0, 0, -100000, 100000),
    "joint5": MotorCalibration(5, 0, 0, -65000, 65000),
    "joint6": MotorCalibration(6, 0, 0, -100000, 130000),
    "gripper": MotorCalibration(7, 0, 0, 0, 68000),
}


def _run_cmd(cmd: list[str], sudo: bool = False) -> tuple[int, str, str]:
    """Run a shell command, optionally with sudo. Returns (returncode, stdout, stderr)."""
    if sudo:
        cmd = ["sudo"] + cmd
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
    return result.returncode, result.stdout.strip(), result.stderr.strip()


def detect_can_interfaces() -> list[dict]:
    """Detect CAN interfaces and their USB bus-info.
    Returns list of {'iface': str, 'bus_info': str, 'state': str, 'bitrate': str}.
    """
    rc, out, _ = _run_cmd(["ip", "-br", "link", "show", "type", "can"])
    if rc != 0 or not out:
        return []

    interfaces = []
    for line in out.splitlines():
        parts = line.split()
        if len(parts) < 2:
            continue
        iface = parts[0]
        state = parts[1]

        # Get bus-info via ethtool
        rc2, out2, _ = _run_cmd(["ethtool", "-i", iface], sudo=True)
        bus_info = ""
        if rc2 == 0:
            for l in out2.splitlines():
                if l.startswith("bus-info:"):
                    bus_info = l.split(":", 1)[1].strip()

        # Get current bitrate
        rc3, out3, _ = _run_cmd(["ip", "-details", "link", "show", iface])
        bitrate = ""
        if rc3 == 0:
            for l in out3.splitlines():
                if "bitrate" in l:
                    for token in l.split():
                        if token.isdigit() and int(token) > 10000:
                            bitrate = token
                            break

        interfaces.append({
            "iface": iface,
            "bus_info": bus_info,
            "state": state,
            "bitrate": bitrate,
        })

    return interfaces


def init_can_interface(iface: str, target_name: str, bitrate: int) -> tuple[bool, str]:
    """Initialize a CAN interface: set bitrate, rename, bring up. Returns (success, message)."""
    messages = []

    # Load gs_usb module
    rc, _, err = _run_cmd(["modprobe", "gs_usb"], sudo=True)
    if rc != 0:
        return False, f"Failed to load gs_usb: {err}"

    # Bring down
    _run_cmd(["ip", "link", "set", iface, "down"], sudo=True)

    # Set bitrate
    rc, _, err = _run_cmd(["ip", "link", "set", iface, "type", "can", "bitrate", str(bitrate)], sudo=True)
    if rc != 0:
        return False, f"Failed to set bitrate: {err}"
    messages.append(f"Bitrate set to {bitrate}")

    # Rename if needed
    if iface != target_name:
        # Check target name doesn't already exist
        rc_chk, _, _ = _run_cmd(["ip", "link", "show", target_name])
        if rc_chk == 0:
            return False, f"Interface '{target_name}' already exists, cannot rename"

        rc, _, err = _run_cmd(["ip", "link", "set", iface, "name", target_name], sudo=True)
        if rc != 0:
            return False, f"Failed to rename {iface} -> {target_name}: {err}"
        messages.append(f"Renamed {iface} -> {target_name}")

    # Bring up
    rc, _, err = _run_cmd(["ip", "link", "set", target_name, "up"], sudo=True)
    if rc != 0:
        return False, f"Failed to bring up {target_name}: {err}"
    messages.append(f"{target_name} is UP")

    return True, "; ".join(messages)


def _read_firmware(iface: str) -> str:
    """Briefly connect to the arm on iface, read firmware version, then disconnect."""
    try:
        piper = C_PiperInterface_V2(iface, judge_flag=False, can_auto_init=False)
        piper.ConnectPort(piper_init=False, start_thread=True)
        try:
            piper.SearchPiperFirmwareVersion()
        except Exception:
            pass
        fw = "—"
        for _ in range(5):
            time.sleep(0.2)
            result = piper.GetPiperFirmwareVersion()
            if isinstance(result, str):
                fw = result
                break
        try:
            piper.DisconnectPort()
        except Exception:
            pass
        return fw
    except Exception:
        return "Error"


class PiperControlUI:
    def __init__(self, port: str = "can0", camera_indices: list[int] | None = None):
        self.port = port
        self.camera_indices = camera_indices or []
        self.bus: PiperMotorsBus | None = None
        self.connected = False
        self.torque_on = False
        self.running = True
        self._connected_port: str | None = None
        self.arm_role: str = "follower"  # "follower" or "leader"

        # Camera
        self.cameras: dict[int, cv2.VideoCapture] = {}

        # Build UI
        self.root = tk.Tk()
        self.root.title("Piper Arm Controller")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # Port selection variable (driven by radio buttons; must be after Tk())
        self.port_var = tk.StringVar(value=self.port)

        geo = _load_geometry("piper-ui")
        if geo:
            self.root.geometry(geo)

        self._build_ui()

        # Start polling thread
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        self.root.columnconfigure(0, weight=1)

        # -- CAN Setup (row 0)
        self._build_can_frame()

        # -- Control buttons (row 1)
        btn_frame = ttk.LabelFrame(self.root, text="Control", padding=8)
        btn_frame.grid(row=1, column=0, sticky="ew", padx=8, pady=(8, 4))

        self.btn_connect = ttk.Button(btn_frame, text="Connect", command=self._on_connect)
        self.btn_connect.pack(side="left", padx=4)

        self.btn_disconnect = ttk.Button(btn_frame, text="Disconnect", command=self._on_disconnect, state="disabled")
        self.btn_disconnect.pack(side="left", padx=4)

        self.btn_torque = ttk.Button(btn_frame, text="Torque ON", command=self._on_torque_toggle, state="disabled")
        self.btn_torque.pack(side="left", padx=4)

        self.btn_parking = ttk.Button(btn_frame, text="Parking", command=self._on_parking, state="disabled")
        self.btn_parking.pack(side="left", padx=4)

        ttk.Separator(btn_frame, orient="vertical").pack(side="left", fill="y", padx=8)

        self.btn_set_leader = ttk.Button(btn_frame, text="Set Leader", command=self._on_set_leader, state="disabled")
        self.btn_set_leader.pack(side="left", padx=4)

        self.btn_set_follower = ttk.Button(btn_frame, text="Set Follower", command=self._on_set_follower, state="disabled")
        self.btn_set_follower.pack(side="left", padx=4)

        # -- Status bar (row 99)
        self.status_var = tk.StringVar(value="Disconnected")
        status_bar = ttk.Label(self.root, textvariable=self.status_var, relief="sunken", anchor="w", padding=4)
        status_bar.grid(row=99, column=0, sticky="ew", padx=8, pady=(4, 8))

        # -- Joint Monitor (row 2)
        self._build_joint_monitor()

        # -- Joints (row 3)
        joint_frame = ttk.LabelFrame(self.root, text="Joints", padding=8)
        joint_frame.grid(row=3, column=0, sticky="ew", padx=8, pady=4)

        self.sliders: dict[str, tk.Scale] = {}
        self.pos_labels: dict[str, tk.StringVar] = {}

        for i, name in enumerate(JOINTS):
            lo, hi = SLIDER_RANGE[name]

            ttk.Label(joint_frame, text=name, width=8, anchor="e").grid(row=i, column=0, padx=(0, 8))

            slider = tk.Scale(
                joint_frame, from_=lo, to=hi, orient="horizontal",
                length=360, resolution=0.5, showvalue=True,
                command=lambda val, n=name: self._on_slider_change(n, val),
                state="disabled",
            )
            slider.set(0)
            slider.grid(row=i, column=1, sticky="ew", padx=4)
            self.sliders[name] = slider

            pos_var = tk.StringVar(value="--")
            ttk.Label(joint_frame, textvariable=pos_var, width=10, anchor="e").grid(row=i, column=2, padx=8)
            self.pos_labels[name] = pos_var

        joint_frame.columnconfigure(1, weight=1)

        # -- Camera feeds (row 4)
        if self.camera_indices:
            cam_frame = ttk.LabelFrame(self.root, text="Cameras", padding=8)
            cam_frame.grid(row=4, column=0, sticky="nsew", padx=8, pady=4)
            self.root.rowconfigure(4, weight=1)

            self.cam_labels: dict[int, ttk.Label] = {}
            for col, idx in enumerate(self.camera_indices):
                lbl = ttk.Label(cam_frame, text=f"Camera {idx}")
                lbl.grid(row=0, column=col, padx=4, pady=4)
                self.cam_labels[idx] = lbl
                cam_frame.columnconfigure(col, weight=1)

    # ------------------------------------------------------ Joint Monitor
    def _build_joint_monitor(self):
        mon_frame = ttk.LabelFrame(self.root, text="Joint Monitor", padding=8)
        mon_frame.grid(row=2, column=0, sticky="ew", padx=8, pady=4)

        self.mon_labels: dict[str, tk.Label] = {}
        for col, name in enumerate(JOINTS):
            ttk.Label(mon_frame, text=name, anchor="center").grid(row=0, column=col, padx=6)
            lbl = tk.Label(
                mon_frame, text="--", font=("monospace", 14, "bold"),
                width=7, anchor="center", relief="sunken", bg="#f0f0f0",
            )
            lbl.grid(row=1, column=col, padx=6, pady=(2, 0))
            self.mon_labels[name] = lbl
            mon_frame.columnconfigure(col, weight=1)

    # --------------------------------------------------------- CAN Setup
    def _build_can_frame(self):
        can_frame = ttk.LabelFrame(self.root, text="CAN Setup", padding=8)
        can_frame.grid(row=0, column=0, sticky="ew", padx=8, pady=(8, 4))

        # Detect / Init All buttons + status
        btn_row = ttk.Frame(can_frame)
        btn_row.grid(row=0, column=0, columnspan=10, sticky="ew", pady=(0, 4))

        ttk.Button(btn_row, text="Detect", command=self._on_can_detect).pack(side="left", padx=4)
        ttk.Button(btn_row, text="Init All", command=self._on_can_init_all).pack(side="left", padx=4)

        self.can_status_var = tk.StringVar(value="Click 'Detect' to scan CAN interfaces")
        ttk.Label(btn_row, textvariable=self.can_status_var).pack(side="left", padx=12)

        # Columns: Sel | Interface | USB Port | State | Bitrate | Firmware | Role | Target Name | Target Bitrate | ""
        headers = ["Sel", "Interface", "USB Port", "State", "Bitrate", "Firmware", "Role", "Target Name", "Target Bitrate", ""]
        for col, h in enumerate(headers):
            ttk.Label(can_frame, text=h, font=("", 9, "bold")).grid(row=1, column=col, padx=4, sticky="w")

        ttk.Separator(can_frame, orient="horizontal").grid(row=2, column=0, columnspan=10, sticky="ew", pady=2)

        # Port rows (populated by detect)
        self.can_rows_frame = ttk.Frame(can_frame)
        self.can_rows_frame.grid(row=3, column=0, columnspan=10, sticky="ew")
        self.can_row_widgets: list[dict] = []

    def _on_can_detect(self):
        for w in self.can_rows_frame.winfo_children():
            w.destroy()
        self.can_row_widgets.clear()

        interfaces = detect_can_interfaces()
        if not interfaces:
            self.can_status_var.set("No CAN interfaces detected")
            return

        self.can_status_var.set(f"{len(interfaces)} interface(s) found")

        # Keep current selection if still valid, else auto-select first
        current = self.port_var.get()
        if current not in [i["iface"] for i in interfaces]:
            self.port_var.set(interfaces[0]["iface"])

        for i, info in enumerate(interfaces):
            row_data: dict = {"info": info}

            # Sel: radio button
            rb = ttk.Radiobutton(self.can_rows_frame, variable=self.port_var, value=info["iface"])
            rb.grid(row=i, column=0, padx=4)
            row_data["radio"] = rb

            ttk.Label(self.can_rows_frame, text=info["iface"]).grid(row=i, column=1, padx=4, sticky="w")
            ttk.Label(self.can_rows_frame, text=info["bus_info"]).grid(row=i, column=2, padx=4, sticky="w")

            state_color = "green" if info["state"] == "UP" else "gray"
            tk.Label(self.can_rows_frame, text=info["state"], fg=state_color).grid(row=i, column=3, padx=4, sticky="w")

            ttk.Label(self.can_rows_frame, text=info["bitrate"] or "--").grid(row=i, column=4, padx=4, sticky="w")

            # Firmware (populated async for UP interfaces)
            fw_var = tk.StringVar(value="..." if info["state"] == "UP" else "—")
            ttk.Label(self.can_rows_frame, textvariable=fw_var, width=14).grid(row=i, column=5, padx=4, sticky="w")
            row_data["fw_var"] = fw_var

            # Role combobox
            default_role = "leader" if i == 0 and len(interfaces) > 1 else "follower"
            role_var = tk.StringVar(value=default_role)
            role_cb = ttk.Combobox(
                self.can_rows_frame, textvariable=role_var,
                values=["follower", "leader"], state="readonly", width=9,
            )
            role_cb.grid(row=i, column=6, padx=4)
            row_data["role_var"] = role_var

            # Target name
            default_name = f"can_{'leader' if i == 0 else 'follower'}" if len(interfaces) > 1 else "can_follower"
            name_var = tk.StringVar(value=default_name)
            ttk.Entry(self.can_rows_frame, textvariable=name_var, width=14).grid(row=i, column=7, padx=4)
            row_data["target_name"] = name_var

            # Target bitrate
            br_var = tk.StringVar(value="1000000")
            ttk.Entry(self.can_rows_frame, textvariable=br_var, width=10).grid(row=i, column=8, padx=4)
            row_data["target_bitrate"] = br_var

            # Init button
            btn = ttk.Button(
                self.can_rows_frame, text="Init",
                command=lambda idx=i: self._on_can_init_single(idx),
            )
            btn.grid(row=i, column=9, padx=4)
            row_data["btn"] = btn

            self.can_row_widgets.append(row_data)

        # Read firmware asynchronously for each UP interface that is not currently in use
        for i, info in enumerate(interfaces):
            if info["state"] != "UP":
                continue
            if info["iface"] == self._connected_port:
                self.can_row_widgets[i]["fw_var"].set("In Use")
                continue
            fw_var = self.can_row_widgets[i]["fw_var"]
            threading.Thread(
                target=self._read_firmware_for_row,
                args=(info["iface"], fw_var),
                daemon=True,
            ).start()

    def _read_firmware_for_row(self, iface: str, fw_var: tk.StringVar) -> None:
        fw = _read_firmware(iface)
        self.root.after(0, lambda: fw_var.set(fw))

    def _on_can_init_single(self, idx: int):
        row = self.can_row_widgets[idx]
        iface = row["info"]["iface"]
        target = row["target_name"].get().strip()
        try:
            bitrate = int(row["target_bitrate"].get().strip())
        except ValueError:
            self.can_status_var.set(f"Invalid bitrate for {iface}")
            return

        self.can_status_var.set(f"Initializing {iface} -> {target}...")
        self.root.update()

        ok, msg = init_can_interface(iface, target, bitrate)
        self.can_status_var.set(f"{'OK' if ok else 'FAIL'}: {msg}")

        if ok:
            self._on_can_detect()  # refresh

    def _on_can_init_all(self):
        if not self.can_row_widgets:
            self.can_status_var.set("No interfaces to init. Click 'Detect' first.")
            return

        results = []
        for row in self.can_row_widgets:
            iface = row["info"]["iface"]
            target = row["target_name"].get().strip()
            try:
                bitrate = int(row["target_bitrate"].get().strip())
            except ValueError:
                results.append(f"{iface}: invalid bitrate")
                continue

            self.can_status_var.set(f"Initializing {iface}...")
            self.root.update()

            ok, _ = init_can_interface(iface, target, bitrate)
            results.append(f"{target}: {'OK' if ok else 'FAIL'}")

        self.can_status_var.set(" | ".join(results))
        self._on_can_detect()  # refresh

    # ------------------------------------------------ Port list enable/disable
    def _set_port_radios_state(self, state: str) -> None:
        for row in self.can_row_widgets:
            if "radio" in row:
                row["radio"].config(state=state)

    # ------------------------------------------------ Role-aware joint read
    def _read_joints(self) -> dict:
        """Read joint positions using the correct CAN address for the connected role."""
        if self.arm_role == "leader":
            return self.bus.get_control()   # CAN 0x155-0x157 (teaching broadcast)
        return self.bus.get_action()        # CAN 0x2A5-0x2A7 (encoder feedback)

    # ------------------------------------------------ Movement controls state
    def _update_movement_controls(self):
        """Enable sliders and Parking only when connected and torque is on."""
        state = "normal" if (self.connected and self.torque_on) else "disabled"
        for slider in self.sliders.values():
            slider.config(state=state)
        self.btn_parking.config(state=state)

    # ------------------------------------------------------------ Actions
    def _on_connect(self):
        port = self.port_var.get().strip()
        if not port:
            self.status_var.set("Error: no port selected. Click 'Detect' first.")
            return

        self.status_var.set(f"Connecting to {port}...")
        self.root.update()

        self.bus = PiperMotorsBus(
            id="ui", port=port,
            motors=DEFAULT_MOTORS,
            calibration=DEFAULT_CALIBRATION,
        )

        try:
            self.bus.connect()
        except Exception as e:
            self.status_var.set(f"Failed to connect: {e}")
            self.bus = None
            return

        self.connected = True
        self._connected_port = port

        # Pick up role from the port list row
        self.arm_role = "follower"
        for row in self.can_row_widgets:
            if row["info"]["iface"] == port:
                self.arm_role = row["role_var"].get()
                break
        self.status_var.set(f"Connected to {port} ({self.arm_role})")

        self.btn_connect.config(state="disabled")
        self.btn_disconnect.config(state="normal")
        self.btn_torque.config(state="normal")
        self.btn_set_leader.config(state="normal")
        self.btn_set_follower.config(state="normal")
        self._set_port_radios_state("disabled")
        self._update_movement_controls()

        # Sync sliders after a short delay so the CAN thread receives the first feedback
        def _sync_sliders():
            time.sleep(1.0)
            if not self.connected or self.bus is None:
                return
            try:
                pos = self._read_joints()
            except Exception as e:
                logger.warning(f"Slider sync read failed: {e}")
                return

            def _apply():
                for n, v in pos.items():
                    if n in self.sliders:
                        s = self.sliders[n]
                        # Briefly enable so tkinter renders the new thumb position
                        s.config(state="normal")
                        s.set(v)
                        if not self.torque_on:
                            s.config(state="disabled")

            self.root.after(0, _apply)

        threading.Thread(target=_sync_sliders, daemon=True).start()

        # Open cameras
        for idx in self.camera_indices:
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                self.cameras[idx] = cap
                logger.info(f"Camera {idx} opened.")

    def _on_disconnect(self):
        if self.bus:
            self.bus.disconnect(disable_torque=self.torque_on)
            self.bus = None

        self.connected = False
        self.torque_on = False
        self._connected_port = None

        # Release cameras
        for cap in self.cameras.values():
            cap.release()
        self.cameras.clear()

        for lbl in self.mon_labels.values():
            lbl.config(text="--")

        self.status_var.set("Disconnected")
        self.btn_connect.config(state="normal")
        self.btn_disconnect.config(state="disabled")
        self.btn_torque.config(state="disabled")
        self.btn_torque.config(text="Torque ON")
        self.btn_set_leader.config(state="disabled")
        self.btn_set_follower.config(state="disabled")
        self._set_port_radios_state("normal")
        self._update_movement_controls()

    def _on_torque_toggle(self):
        if not self.bus:
            return
        if not self.torque_on:
            try:
                self.bus.enable_torque()
            except Exception as e:
                self.status_var.set(f"Torque failed: {e}")
                return
            self.torque_on = True
            self.btn_torque.config(text="Torque OFF")
            self.status_var.set("Torque enabled")
        else:
            self.bus.disable_torque()
            self.torque_on = False
            self.btn_torque.config(text="Torque ON")
            self.status_var.set("Torque disabled")
        self._update_movement_controls()

    def _on_parking(self):
        if not self.bus or not self.torque_on:
            return
        self.status_var.set("Parking...")
        self.root.update()
        self.bus.parking()
        for name, val in INITIALIZE_POSITION.items():
            if name in self.sliders:
                self.sliders[name].set(val)
        self.status_var.set("Parked")

    def _on_slider_change(self, name: str, _val: str):
        if not self.connected or not self.bus or not self.torque_on:
            return
        goal = {n: float(self.sliders[n].get()) for n in JOINTS}
        try:
            self.bus.set_action(goal, is_conv=True)
        except Exception:
            logger.exception(f"Failed to send slider value for {name}")

    def _on_set_leader(self):
        if not self.bus:
            return
        self.btn_set_leader.config(state="disabled")

        def worker():
            try:
                self.bus.set_master()
                self.root.after(0, lambda: self.status_var.set("Set as Leader (teaching input mode)"))
            except Exception as e:
                self.root.after(0, lambda: self.status_var.set(f"Set Leader failed: {e}"))
            finally:
                self.root.after(0, lambda: self.btn_set_leader.config(state="normal"))

        threading.Thread(target=worker, daemon=True).start()

    def _on_set_follower(self):
        if not self.bus:
            return
        self.btn_set_follower.config(state="disabled")

        def worker():
            try:
                self.bus.set_slave()
                self.root.after(0, lambda: self.status_var.set("Set as Follower (motion output mode)"))
            except Exception as e:
                self.root.after(0, lambda: self.status_var.set(f"Set Follower failed: {e}"))
            finally:
                self.root.after(0, lambda: self.btn_set_follower.config(state="normal"))

        threading.Thread(target=worker, daemon=True).start()

    # --------------------------------------------------------- Poll loop
    def _poll_loop(self):
        while self.running:
            if not self.connected or not self.bus:
                time.sleep(0.1)
                continue

            try:
                pos = self._read_joints()
                for name, val in pos.items():
                    if name in self.pos_labels:
                        self.pos_labels[name].set(f"{val:.1f}")
                    if name in self.mon_labels:
                        self.mon_labels[name].config(text=f"{val:.1f}")

                for idx, cap in list(self.cameras.items()):
                    ret, frame = cap.read()
                    if ret and idx in getattr(self, "cam_labels", {}):
                        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        frame_rgb = cv2.resize(frame_rgb, (320, 240))
                        img = Image.fromarray(frame_rgb)
                        imgtk = ImageTk.PhotoImage(image=img)
                        self.cam_labels[idx].config(image=imgtk, text="")
                        self.cam_labels[idx].imgtk = imgtk

            except Exception:
                logger.exception("Poll error")

            time.sleep(0.05)  # ~20 Hz

    # -------------------------------------------------------------- Run
    def _on_close(self):
        _save_geometry("piper-ui", self.root.geometry())
        self.running = False
        if self.connected:
            self._on_disconnect()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Piper Arm Control UI")
    parser.add_argument("--port", default="can0", help="CAN port (default: can0)")
    parser.add_argument("--cameras", type=int, nargs="*", default=[], help="Camera indices (e.g. 0 2)")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO)
    app = PiperControlUI(port=args.port, camera_indices=args.cameras)
    app.run()


if __name__ == "__main__":
    main()
