"""
calibration_ui.py — Piper joint zero-point calibration tool

Workflow:
  1. Connect to a Piper arm via CAN
  2. Manually move each joint to its zero (home) position
  3. Click "Set Zero" to call JointConfig(set_zero=0xAE) for each joint
  4. SDK writes zero-point to the motor's flash — persists across power cycles
  5. Real-time joint monitor shows current raw positions for verification

Entry point: piper-calibrate
"""

from __future__ import annotations

import logging
import subprocess
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

from piper_sdk import C_PiperInterface_V2

from .ui import _load_geometry, _save_geometry

logger = logging.getLogger(__name__)

JOINTS_MOTOR = {
    "joint1": 1,
    "joint2": 2,
    "joint3": 3,
    "joint4": 4,
    "joint5": 5,
    "joint6": 6,
}

GRIPPER_MOTOR = 7

ALL_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]

# ─────────────────────────────── CAN helpers ─────────────────────────────────


def _run_cmd(cmd: list[str], sudo: bool = False) -> tuple[int, str, str]:
    if sudo:
        cmd = ["sudo"] + cmd
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
    return result.returncode, result.stdout.strip(), result.stderr.strip()


def detect_can_interfaces() -> list[str]:
    rc, out, _ = _run_cmd(["ip", "-br", "link", "show", "type", "can"])
    if rc != 0 or not out:
        return []
    ifaces = []
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 2:
            ifaces.append(parts[0])
    return ifaces


def init_can_interface(iface: str, bitrate: int) -> tuple[bool, str]:
    _run_cmd(["modprobe", "gs_usb"], sudo=True)
    _run_cmd(["ip", "link", "set", iface, "down"], sudo=True)
    rc, _, err = _run_cmd(
        ["ip", "link", "set", iface, "type", "can", "bitrate", str(bitrate)], sudo=True
    )
    if rc != 0:
        return False, f"set bitrate failed: {err}"
    rc, _, err = _run_cmd(["ip", "link", "set", iface, "up"], sudo=True)
    if rc != 0:
        return False, f"bring-up failed: {err}"
    return True, "OK"


# ─────────────────────────────── main UI ─────────────────────────────────────


class CalibrationUI:
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("Piper Joint Calibration (Zero-Point)")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.minsize(700, 480)

        geo = _load_geometry("piper-calibrate")
        if geo:
            self.root.geometry(geo)

        self.piper: C_PiperInterface_V2 | None = None
        self._connected = False
        self._polling = False
        self._running = True

        # Current raw positions (updated by poll thread)
        self._raw: dict[str, int] = {j: 0 for j in ALL_JOINTS}

        self._build_ui()

    # ─────────────────────────── UI construction ──────────────────────────────

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)

        # ── Connection frame ──
        conn_frame = ttk.LabelFrame(self.root, text="Connection", padding=8)
        conn_frame.grid(row=0, column=0, sticky="ew", padx=8, pady=(8, 4))

        ttk.Label(conn_frame, text="CAN Port:").pack(side="left", padx=4)
        self._port_var = tk.StringVar(value="can0")
        self._port_combo = ttk.Combobox(
            conn_frame, textvariable=self._port_var, width=14, state="readonly"
        )
        self._port_combo.pack(side="left", padx=4)

        ttk.Button(conn_frame, text="Detect", command=self._on_detect).pack(side="left", padx=4)

        self._btn_connect = ttk.Button(conn_frame, text="Connect", command=self._on_connect)
        self._btn_connect.pack(side="left", padx=4)

        self._btn_disconnect = ttk.Button(
            conn_frame, text="Disconnect", command=self._on_disconnect, state="disabled"
        )
        self._btn_disconnect.pack(side="left", padx=4)

        self._conn_status = tk.StringVar(value="Disconnected")
        ttk.Label(conn_frame, textvariable=self._conn_status).pack(side="left", padx=12)

        self._on_detect()

        # ── Instructions ──
        info_frame = ttk.LabelFrame(self.root, text="Instructions", padding=8)
        info_frame.grid(row=1, column=0, sticky="ew", padx=8, pady=4)

        info_text = (
            "1. Connect to the arm (teaching/free mode recommended)\n"
            "2. Manually move each joint to the desired zero (home) position\n"
            "3. Click 'Set Zero' for each joint, or 'Set All Zero' for all at once\n"
            "4. The zero-point is saved to motor flash — persists across power cycles"
        )
        ttk.Label(info_frame, text=info_text, justify="left").pack(anchor="w")

        # ── Joint table ──
        table_frame = ttk.LabelFrame(self.root, text="Joint Positions & Zero-Point", padding=8)
        table_frame.grid(row=2, column=0, sticky="nsew", padx=8, pady=4)
        self.root.rowconfigure(2, weight=1)

        headers = ["Joint", "Motor #", "Current Raw", "Position Bar", "Set Zero", "Status"]
        col_widths = [8, 7, 12, 0, 10, 14]
        for col, (h, _w) in enumerate(zip(headers, col_widths)):
            ttk.Label(table_frame, text=h, font=("", 9, "bold"), anchor="center").grid(
                row=0, column=col, padx=6, pady=(0, 4)
            )
        table_frame.columnconfigure(3, weight=1)

        ttk.Separator(table_frame, orient="horizontal").grid(
            row=1, column=0, columnspan=6, sticky="ew", pady=2
        )

        self._cur_vars: dict[str, tk.StringVar] = {}
        self._bars: dict[str, tk.Canvas] = {}
        self._zero_btns: dict[str, ttk.Button] = {}
        self._status_labels: dict[str, tk.StringVar] = {}

        for i, name in enumerate(ALL_JOINTS):
            r = i + 2
            motor_num = JOINTS_MOTOR.get(name, GRIPPER_MOTOR)

            # Joint name
            ttk.Label(table_frame, text=name, width=8, anchor="e",
                      font=("", 10, "bold")).grid(row=r, column=0, padx=6, pady=3)

            # Motor number
            ttk.Label(table_frame, text=str(motor_num), width=7,
                      anchor="center").grid(row=r, column=1, padx=6, pady=3)

            # Current raw value
            cv = tk.StringVar(value="--")
            tk.Label(table_frame, textvariable=cv, width=12, anchor="center",
                     font=("monospace", 13, "bold"), relief="sunken", bg="#f0f0f0").grid(
                row=r, column=2, padx=6, pady=3
            )
            self._cur_vars[name] = cv

            # Position bar
            bar = tk.Canvas(table_frame, height=22, bg="#e8e8e8", highlightthickness=1,
                            highlightbackground="#ccc")
            bar.grid(row=r, column=3, padx=6, pady=3, sticky="ew")
            self._bars[name] = bar

            # Set Zero button
            btn = ttk.Button(table_frame, text="Set Zero",
                             command=lambda n=name: self._on_set_zero(n),
                             state="disabled")
            btn.grid(row=r, column=4, padx=6, pady=3)
            self._zero_btns[name] = btn

            # Status
            sv = tk.StringVar(value="--")
            ttk.Label(table_frame, textvariable=sv, width=14, anchor="center").grid(
                row=r, column=5, padx=6, pady=3
            )
            self._status_labels[name] = sv

        # ── Bottom controls ──
        bottom_frame = ttk.Frame(self.root, padding=8)
        bottom_frame.grid(row=3, column=0, sticky="ew", padx=8, pady=4)

        self._btn_set_all = ttk.Button(
            bottom_frame, text="Set All Zero", command=self._on_set_all_zero, state="disabled"
        )
        self._btn_set_all.pack(side="left", padx=4)

        self._btn_clear_err = ttk.Button(
            bottom_frame, text="Clear All Errors", command=self._on_clear_all_errors, state="disabled"
        )
        self._btn_clear_err.pack(side="left", padx=4)

        # ── Status bar ──
        self._status_var = tk.StringVar(value="Connect to an arm, then set zero-points.")
        ttk.Label(self.root, textvariable=self._status_var, relief="sunken",
                  anchor="w", padding=4).grid(row=99, column=0, sticky="ew", padx=8, pady=(4, 8))

    # ─────────────────────────── connection ───────────────────────────────────

    def _on_detect(self) -> None:
        ifaces = detect_can_interfaces()
        if ifaces:
            self._port_combo["values"] = ifaces
            if self._port_var.get() not in ifaces:
                self._port_var.set(ifaces[0])
            self._conn_status.set(f"{len(ifaces)} CAN port(s)")
        else:
            self._port_combo["values"] = []
            self._conn_status.set("No CAN ports found")

    def _on_connect(self) -> None:
        port = self._port_var.get().strip()
        if not port:
            self._conn_status.set("No port selected")
            return

        self._conn_status.set(f"Connecting to {port}...")
        self.root.update()

        ok, msg = init_can_interface(port, 1000000)
        if not ok:
            self._conn_status.set(f"Init failed: {msg}")
            return

        try:
            piper = C_PiperInterface_V2(port, judge_flag=False, can_auto_init=False)
            piper.CreateCanBus(port)
            piper.ConnectPort(piper_init=False, start_thread=True)
            time.sleep(0.5)
        except Exception as e:
            self._conn_status.set(f"Connect failed: {e}")
            return

        self.piper = piper
        self._connected = True
        self._polling = True

        self._btn_connect.config(state="disabled")
        self._btn_disconnect.config(state="normal")
        self._port_combo.config(state="disabled")
        self._conn_status.set(f"Connected to {port}")

        # Enable zero buttons
        for btn in self._zero_btns.values():
            btn.config(state="normal")
        self._btn_set_all.config(state="normal")
        self._btn_clear_err.config(state="normal")

        self._status_var.set("Connected. Move joints to home position, then click Set Zero.")

        threading.Thread(target=self._poll_loop, daemon=True).start()
        self._refresh_ui()

    def _on_disconnect(self) -> None:
        self._polling = False
        time.sleep(0.1)

        if self.piper:
            try:
                self.piper.DisconnectPort()
            except Exception:
                pass
            self.piper = None

        self._connected = False

        self._btn_connect.config(state="normal")
        self._btn_disconnect.config(state="disabled")
        self._port_combo.config(state="readonly")
        self._conn_status.set("Disconnected")

        for btn in self._zero_btns.values():
            btn.config(state="disabled")
        self._btn_set_all.config(state="disabled")
        self._btn_clear_err.config(state="disabled")

        for name in ALL_JOINTS:
            self._cur_vars[name].set("--")
            self._status_labels[name].set("--")

    # ─────────────────────────── polling ──────────────────────────────────────

    def _poll_loop(self) -> None:
        while self._polling and self._running:
            if not self.piper:
                time.sleep(0.1)
                continue
            try:
                msg_j = self.piper.GetArmJointMsgs()
                msg_g = self.piper.GetArmGripperMsgs()
                self._raw = {
                    "joint1": int(msg_j.joint_state.joint_1),
                    "joint2": int(msg_j.joint_state.joint_2),
                    "joint3": int(msg_j.joint_state.joint_3),
                    "joint4": int(msg_j.joint_state.joint_4),
                    "joint5": int(msg_j.joint_state.joint_5),
                    "joint6": int(msg_j.joint_state.joint_6),
                    "gripper": int(msg_g.gripper_state.grippers_angle),
                }
            except Exception:
                logger.exception("Poll error")
            time.sleep(0.05)

    def _refresh_ui(self) -> None:
        if not self._running:
            return

        for name in ALL_JOINTS:
            v = self._raw.get(name, 0)
            self._cur_vars[name].set(str(v))
            self._draw_bar(name, v)

        self.root.after(50, self._refresh_ui)

    def _draw_bar(self, name: str, current: int) -> None:
        canvas = self._bars[name]
        canvas.delete("all")

        w = canvas.winfo_width()
        h = canvas.winfo_height()
        if w < 10:
            return

        # Show position relative to zero — zero is center
        # Scale: full bar = +/- 180 deg = +/- 180000 raw units
        scale = 180000 if name != "gripper" else 70000
        mid = w / 2

        # Zero line (center)
        canvas.create_line(mid, 0, mid, h, fill="#888", width=1, dash=(2, 2))

        # Current position
        offset = (current / scale) * (w / 2)
        cx = mid + offset
        cx = max(2, min(w - 2, cx))

        # Color: green near zero, yellow/red further away
        abs_ratio = min(abs(current) / scale, 1.0)
        if abs_ratio < 0.05:
            color = "#00cc00"  # green — close to zero
        elif abs_ratio < 0.3:
            color = "#ffaa00"  # yellow
        else:
            color = "#cc3300"  # red

        # Draw position indicator
        bar_w = max(4, int(w * 0.02))
        canvas.create_rectangle(cx - bar_w // 2, 1, cx + bar_w // 2, h - 1,
                                fill=color, outline="")

        # Value near zero indicator
        if abs(current) < scale * 0.02:
            canvas.create_text(w - 4, h // 2, text="OK", anchor="e",
                               fill="#00aa00", font=("", 8, "bold"))

    # ─────────────────────────── zero-point actions ───────────────────────────

    def _on_set_zero(self, name: str) -> None:
        if not self.piper:
            return

        current_val = self._raw.get(name, 0)
        motor_num = JOINTS_MOTOR.get(name, GRIPPER_MOTOR)

        if abs(current_val) > 5000 and name != "gripper":
            ok = messagebox.askyesno(
                "Confirm Set Zero",
                f"{name} (motor {motor_num}) current value is {current_val}.\n"
                f"This is relatively far from zero.\n\n"
                f"Are you sure you want to set this position as the new zero-point?"
            )
            if not ok:
                return

        self._zero_btns[name].config(state="disabled")
        self._status_labels[name].set("Setting...")
        self.root.update()

        def worker():
            try:
                if name == "gripper":
                    # Gripper uses GripperCtrl with set_zero=0xAE
                    self.piper.GripperCtrl(0, 1000, 0x01, 0xAE)
                else:
                    # Joint uses JointConfig with set_zero=0xAE
                    self.piper.JointConfig(joint_num=motor_num, set_zero=0xAE)

                time.sleep(0.5)

                # Check response
                resp = self.piper.GetRespInstruction()
                flag = resp.instruction_response.is_set_zero_successfully
                if flag == 1:
                    result = "OK"
                elif flag == 0:
                    result = "Failed"
                else:
                    result = "Sent"  # no explicit response but command was sent

                self.root.after(0, lambda: self._status_labels[name].set(result))
                self.root.after(0, lambda: self._status_var.set(
                    f"{name} (motor {motor_num}): zero-point set — {result}"))
            except Exception as e:
                self.root.after(0, lambda: self._status_labels[name].set("Error"))
                self.root.after(0, lambda: self._status_var.set(f"{name}: {e}"))
            finally:
                self.root.after(0, lambda: self._zero_btns[name].config(state="normal"))

        threading.Thread(target=worker, daemon=True).start()

    def _on_set_all_zero(self) -> None:
        if not self.piper:
            return

        ok = messagebox.askyesno(
            "Set All Zero",
            "This will set the current position of ALL joints and gripper as zero-point.\n\n"
            "Make sure all joints are at the desired home position.\n\n"
            "Continue?"
        )
        if not ok:
            return

        for name in ALL_JOINTS:
            self._on_set_zero(name)
            time.sleep(0.3)  # small delay between commands

    def _on_clear_all_errors(self) -> None:
        if not self.piper:
            return

        self._status_var.set("Clearing errors...")
        self.root.update()

        try:
            for motor_num in range(1, 7):
                self.piper.JointConfig(joint_num=motor_num, clear_err=0xAE)
                time.sleep(0.1)
            self._status_var.set("All joint errors cleared")
        except Exception as e:
            self._status_var.set(f"Clear errors failed: {e}")

    # ─────────────────────────── close ────────────────────────────────────────

    def _on_close(self) -> None:
        _save_geometry("piper-calibrate", self.root.geometry())
        self._running = False
        if self._connected:
            self._on_disconnect()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    logging.basicConfig(level=logging.INFO)
    app = CalibrationUI()
    app.run()


if __name__ == "__main__":
    main()
