import logging
import subprocess
import threading
import time
import tkinter as tk
from tkinter import ttk

import cv2
from PIL import Image, ImageTk

from lerobot.motors import Motor, MotorCalibration, MotorNormMode

from .motors import PiperMotorsBus

logger = logging.getLogger(__name__)

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


class PiperControlUI:
    def __init__(self, port: str = "can0", camera_indices: list[int] | None = None):
        self.port = port
        self.camera_indices = camera_indices or []
        self.bus: PiperMotorsBus | None = None
        self.connected = False
        self.torque_on = False
        self.running = True

        # Camera
        self.cameras: dict[int, cv2.VideoCapture] = {}

        # Build UI
        self.root = tk.Tk()
        self.root.title("Piper Arm Controller")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._build_ui()

        # Start polling thread
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        self.root.columnconfigure(0, weight=1)

        # -- CAN Setup
        self._build_can_frame()

        # -- Top: Control buttons
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

        # Port entry
        ttk.Label(btn_frame, text="Port:").pack(side="left", padx=(16, 2))
        self.port_var = tk.StringVar(value=self.port)
        port_entry = ttk.Entry(btn_frame, textvariable=self.port_var, width=12)
        port_entry.pack(side="left", padx=2)

        # -- Status bar
        self.status_var = tk.StringVar(value="Disconnected")
        status_bar = ttk.Label(self.root, textvariable=self.status_var, relief="sunken", anchor="w", padding=4)
        status_bar.grid(row=99, column=0, sticky="ew", padx=8, pady=(4, 8))

        # -- Middle: Sliders + Position
        joint_frame = ttk.LabelFrame(self.root, text="Joints", padding=8)
        joint_frame.grid(row=2, column=0, sticky="ew", padx=8, pady=4)

        self.sliders: dict[str, tk.Scale] = {}
        self.pos_labels: dict[str, tk.StringVar] = {}

        for i, name in enumerate(JOINTS):
            lo, hi = SLIDER_RANGE[name]

            ttk.Label(joint_frame, text=name, width=8, anchor="e").grid(row=i, column=0, padx=(0, 8))

            slider = tk.Scale(
                joint_frame, from_=lo, to=hi, orient="horizontal",
                length=360, resolution=0.5, showvalue=True,
                command=lambda val, n=name: self._on_slider_change(n, val),
            )
            slider.set(0)
            slider.grid(row=i, column=1, sticky="ew", padx=4)
            self.sliders[name] = slider

            pos_var = tk.StringVar(value="--")
            ttk.Label(joint_frame, textvariable=pos_var, width=10, anchor="e").grid(row=i, column=2, padx=8)
            self.pos_labels[name] = pos_var

        joint_frame.columnconfigure(1, weight=1)

        # -- Camera feeds
        if self.camera_indices:
            cam_frame = ttk.LabelFrame(self.root, text="Cameras", padding=8)
            cam_frame.grid(row=3, column=0, sticky="nsew", padx=8, pady=4)
            self.root.rowconfigure(3, weight=1)

            self.cam_labels: dict[int, ttk.Label] = {}
            for col, idx in enumerate(self.camera_indices):
                lbl = ttk.Label(cam_frame, text=f"Camera {idx}")
                lbl.grid(row=0, column=col, padx=4, pady=4)
                self.cam_labels[idx] = lbl
                cam_frame.columnconfigure(col, weight=1)

    # --------------------------------------------------------- CAN Setup
    def _build_can_frame(self):
        can_frame = ttk.LabelFrame(self.root, text="CAN Setup", padding=8)
        can_frame.grid(row=0, column=0, sticky="ew", padx=8, pady=(8, 4))
        can_frame.columnconfigure(1, weight=1)

        # Detect button
        btn_row = ttk.Frame(can_frame)
        btn_row.grid(row=0, column=0, columnspan=4, sticky="ew", pady=(0, 4))

        ttk.Button(btn_row, text="Detect", command=self._on_can_detect).pack(side="left", padx=4)
        ttk.Button(btn_row, text="Init All", command=self._on_can_init_all).pack(side="left", padx=4)

        self.can_status_var = tk.StringVar(value="Click 'Detect' to scan CAN interfaces")
        ttk.Label(btn_row, textvariable=self.can_status_var).pack(side="left", padx=12)

        # Header
        headers = ["Interface", "USB Port", "State", "Bitrate", "Target Name", "Target Bitrate", ""]
        for col, h in enumerate(headers):
            ttk.Label(can_frame, text=h, font=("", 9, "bold")).grid(row=1, column=col, padx=4, sticky="w")

        ttk.Separator(can_frame, orient="horizontal").grid(row=2, column=0, columnspan=7, sticky="ew", pady=2)

        # Rows (will be populated by detect)
        self.can_rows_frame = ttk.Frame(can_frame)
        self.can_rows_frame.grid(row=3, column=0, columnspan=7, sticky="ew")
        self.can_row_widgets: list[dict] = []

    def _on_can_detect(self):
        # Clear old rows
        for w in self.can_rows_frame.winfo_children():
            w.destroy()
        self.can_row_widgets.clear()

        interfaces = detect_can_interfaces()
        if not interfaces:
            self.can_status_var.set("No CAN interfaces detected")
            return

        self.can_status_var.set(f"{len(interfaces)} interface(s) found")

        for i, info in enumerate(interfaces):
            row_data = {}
            row_data["info"] = info

            ttk.Label(self.can_rows_frame, text=info["iface"]).grid(row=i, column=0, padx=4, sticky="w")
            ttk.Label(self.can_rows_frame, text=info["bus_info"]).grid(row=i, column=1, padx=4, sticky="w")

            state_color = "green" if info["state"] == "UP" else "gray"
            state_lbl = tk.Label(self.can_rows_frame, text=info["state"], fg=state_color)
            state_lbl.grid(row=i, column=2, padx=4, sticky="w")

            ttk.Label(self.can_rows_frame, text=info["bitrate"] or "--").grid(row=i, column=3, padx=4, sticky="w")

            # Target name (editable)
            default_name = f"can_{'leader' if i == 0 else 'follower'}" if len(interfaces) > 1 else "can_follower"
            name_var = tk.StringVar(value=default_name)
            ttk.Entry(self.can_rows_frame, textvariable=name_var, width=14).grid(row=i, column=4, padx=4)
            row_data["target_name"] = name_var

            # Target bitrate
            br_var = tk.StringVar(value="1000000")
            ttk.Entry(self.can_rows_frame, textvariable=br_var, width=10).grid(row=i, column=5, padx=4)
            row_data["target_bitrate"] = br_var

            # Individual init button
            btn = ttk.Button(
                self.can_rows_frame, text="Init",
                command=lambda idx=i: self._on_can_init_single(idx),
            )
            btn.grid(row=i, column=6, padx=4)
            row_data["btn"] = btn

            self.can_row_widgets.append(row_data)

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

    # ------------------------------------------------------------ Actions
    def _on_connect(self):
        port = self.port_var.get().strip()
        if not port:
            self.status_var.set("Error: port is empty")
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
        self.status_var.set(f"Connected to {port}")

        self.btn_connect.config(state="disabled")
        self.btn_disconnect.config(state="normal")
        self.btn_torque.config(state="normal")
        self.btn_parking.config(state="normal")

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

        # Release cameras
        for cap in self.cameras.values():
            cap.release()
        self.cameras.clear()

        self.status_var.set("Disconnected")
        self.btn_connect.config(state="normal")
        self.btn_disconnect.config(state="disabled")
        self.btn_torque.config(state="disabled")
        self.btn_parking.config(state="disabled")
        self.btn_torque.config(text="Torque ON")

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

    def _on_parking(self):
        if not self.bus:
            return
        self.status_var.set("Parking...")
        self.root.update()
        self.bus.parking()
        # Sync sliders to 0
        for s in self.sliders.values():
            s.set(0)
        self.status_var.set("Parked")

    def _on_slider_change(self, name: str, _val: str):
        """Called immediately when any slider is dragged."""
        if not self.connected or not self.bus or not self.torque_on:
            return
        goal = {n: float(self.sliders[n].get()) for n in JOINTS}
        try:
            self.bus.set_action(goal, is_conv=True)
        except Exception:
            logger.exception(f"Failed to send slider value for {name}")

    # --------------------------------------------------------- Poll loop
    def _poll_loop(self):
        while self.running:
            if not self.connected or not self.bus:
                time.sleep(0.1)
                continue

            try:
                # Read current positions
                pos = self.bus.get_action()
                for name, val in pos.items():
                    if name in self.pos_labels:
                        self.pos_labels[name].set(f"{val:.1f}")

                # Update camera feeds
                for idx, cap in list(self.cameras.items()):
                    ret, frame = cap.read()
                    if ret and idx in getattr(self, "cam_labels", {}):
                        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        frame_rgb = cv2.resize(frame_rgb, (320, 240))
                        img = Image.fromarray(frame_rgb)
                        imgtk = ImageTk.PhotoImage(image=img)
                        self.cam_labels[idx].config(image=imgtk, text="")
                        self.cam_labels[idx].imgtk = imgtk  # keep reference

            except Exception:
                logger.exception("Poll error")

            time.sleep(0.05)  # ~20Hz

    # -------------------------------------------------------------- Run
    def _on_close(self):
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
