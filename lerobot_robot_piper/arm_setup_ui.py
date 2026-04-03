"""
arm_setup_ui.py — Piper multi-arm setup wizard

Workflow (based on FIND-ARM.md):
  Step 1 — Detect all CAN ports, initialize, and verify Piper arms.
            Auto-detect leader/follower mode via SDK.
  Step 2 — Select arm configuration
            (1 Leader/1 Follower, 2 Followers, 2 Leaders, 2 Leaders/2 Followers)
  Step 3 — Click [Find] for each slot and move the target arm more than 45 deg.
            The arm with the largest movement among unassigned arms is assigned.
  Step 4 — Rename CAN ports to canonical names and save config.

Entry point: piper-setup
"""

from __future__ import annotations

import json
import os
import subprocess
import threading
import time
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk

from piper_sdk import C_PiperInterface_V2


# ─────────────────────────────── constants ────────────────────────────────

# Movement detection threshold: >= 45 deg (raw unit: 0.001 deg)
FIND_THRESHOLD_RAW = 45_000
# Find timeout in seconds
FIND_TIMEOUT_SEC = 30

CONFIGS: dict[str, list[str]] = {
    "1 Leader / 1 Follower": ["leader_1", "follower_1"],
    "2 Followers":           ["follower_1", "follower_2"],
    "2 Leaders":             ["leader_1", "leader_2"],
    "2 Leaders / 2 Followers": ["leader_1", "leader_2", "follower_1", "follower_2"],
}


def slot_to_can_name(slot: str) -> str:
    """'leader_1'  →  'can_leader1',   'follower_2'  →  'can_follower2'"""
    role, num = slot.rsplit("_", 1)
    return f"can_{role}{num}"


def slot_to_label(slot: str) -> str:
    """'leader_1'  →  'Leader 1',   'follower_2'  →  'Follower 2'"""
    role, num = slot.rsplit("_", 1)
    return f"{role.capitalize()} {num}"


# ─────────────────────────────── helpers ──────────────────────────────────


def _run_cmd(cmd: list[str], sudo: bool = False) -> tuple[int, str, str]:
    if sudo:
        cmd = ["sudo", "-n"] + cmd
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=8)
        return r.returncode, r.stdout.strip(), r.stderr.strip()
    except Exception as exc:
        return -1, "", str(exc)


def detect_can_interfaces() -> list[dict]:
    """Return list of CAN interfaces on this system (iface / bus_info / state)."""
    rc, out, _ = _run_cmd(["ip", "-br", "link", "show", "type", "can"])
    if rc != 0 or not out.strip():
        return []
    result: list[dict] = []
    for line in out.splitlines():
        parts = line.split()
        if len(parts) < 2:
            continue
        iface, state = parts[0], parts[1]
        bus_info = "—"
        rc2, out2, _ = _run_cmd(["ethtool", "-i", iface], sudo=True)
        if rc2 == 0:
            for ln in out2.splitlines():
                if ln.strip().startswith("bus-info:"):
                    bus_info = ln.split(":", 1)[1].strip()
                    break
        result.append({"iface": iface, "bus_info": bus_info, "state": state})
    return result


def init_can_interface(iface: str, bitrate: int) -> tuple[bool, str]:
    """Initialize a CAN interface at the given bitrate and bring it UP."""
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


def rename_can_interface(old_name: str, new_name: str) -> tuple[bool, str]:
    """Rename a CAN interface and bring it back UP."""
    _run_cmd(["ip", "link", "set", old_name, "down"], sudo=True)
    rc, _, err = _run_cmd(["ip", "link", "set", old_name, "name", new_name], sudo=True)
    if rc != 0:
        return False, f"rename failed: {err}"
    rc, _, err = _run_cmd(["ip", "link", "set", new_name, "up"], sudo=True)
    if rc != 0:
        return False, f"bring-up failed: {err}"
    return True, "OK"


# ─────────────────────────────── data model ───────────────────────────────


class DiscoveredArm:
    """One Piper arm found during the scan step."""

    def __init__(self, iface: str, bus_info: str) -> None:
        self.iface = iface          # current CAN interface name (e.g. "can0")
        self.bus_info = bus_info    # physical USB port path
        self.piper: C_PiperInterface_V2 | None = None
        self.fw = "-"               # firmware version
        self.ctrl_mode_str = "-"    # raw ctrl_mode name from GetArmStatus()
        self.role = "unknown"       # determined by assigned slot, not by ctrl_mode
        self.slot: str | None = None       # assigned slot (e.g. "leader_1")
        self.new_name: str | None = None   # new CAN name (e.g. "can_leader1")
        self._lock = threading.Lock()

    def connect_and_verify(self, bitrate: int) -> bool:
        """
        Initialize the CAN port and connect via SDK.
        Verify this is a Piper arm and detect leader/follower mode.
        """
        ok, _ = init_can_interface(self.iface, bitrate)
        if not ok:
            return False

        try:
            piper = C_PiperInterface_V2(self.iface, judge_flag=False, can_auto_init=False)
            piper.ConnectPort(piper_init=False, start_thread=True)

            # Send firmware query command (not sent when piper_init=False)
            try:
                piper.SearchPiperFirmwareVersion()
            except Exception:
                pass

            # Wait for async firmware CAN response (up to 1 s)
            for _ in range(5):
                time.sleep(0.2)
                fw = piper.GetPiperFirmwareVersion()
                if isinstance(fw, str):
                    self.fw = fw
                    break
            else:
                # No firmware response — verify by reading status messages
                try:
                    piper.GetArmStatus()
                    piper.GetArmJointMsgs()
                except Exception:
                    piper.DisconnectPort()
                    return False

            with self._lock:
                self.piper = piper

            # Read ctrl_mode for display; 0x06 = Linkage_teaching_input_mode (leader)
            try:
                ctrl_mode = piper.GetArmStatus().arm_status.ctrl_mode
                mode_int = ctrl_mode.value if hasattr(ctrl_mode, "value") else int(ctrl_mode)
                # Store human-readable name
                mode_names = {
                    0x00: "Standby", 0x01: "CAN ctrl", 0x02: "Teaching",
                    0x03: "Ethernet", 0x04: "WiFi", 0x05: "Remote ctrl",
                    0x06: "Linkage teaching", 0x07: "Offline trajectory",
                }
                self.ctrl_mode_str = mode_names.get(mode_int, f"0x{mode_int:02X}")
                self.role = "leader" if mode_int == 0x06 else "follower"
            except Exception:
                self.ctrl_mode_str = "?"
                self.role = "follower"

            return True
        except Exception:
            return False

    def read_joints_raw(self) -> list[int] | None:
        """
        Return joint positions in raw units (for movement detection).

        Reads both addresses and returns whichever has non-zero values:
          - GetArmJointCtrl()  (CAN 0x155-0x157): leader teaching broadcast
          - GetArmJointMsgs()  (CAN 0x2A5-0x2A7): encoder feedback

        Does not rely on detected role, so detection works even when
        the arm is in Standby mode and role was misidentified.
        """
        with self._lock:
            if not self.piper:
                return None
            ctrl_vals = None
            state_vals = None
            try:
                j = self.piper.GetArmJointCtrl().joint_ctrl
                ctrl_vals = [j.joint_1, j.joint_2, j.joint_3,
                             j.joint_4, j.joint_5, j.joint_6]
            except Exception:
                pass
            try:
                j = self.piper.GetArmJointMsgs().joint_state
                state_vals = [j.joint_1, j.joint_2, j.joint_3,
                              j.joint_4, j.joint_5, j.joint_6]
            except Exception:
                pass

            # Prefer ctrl if non-zero (teaching mode), otherwise use state
            if ctrl_vals and any(v != 0 for v in ctrl_vals):
                return ctrl_vals
            return state_vals

    def disconnect(self) -> None:
        with self._lock:
            if self.piper:
                try:
                    self.piper.DisconnectPort()
                except Exception:
                    pass
                self.piper = None


# ─────────────────────────────── main UI ──────────────────────────────────


class ArmSetupUI:
    """
    4-step setup wizard

    Step 1 | Scan CAN ports -> verify Piper -> detect leader/follower
    Step 2 | Select configuration
    Step 3 | [Find] button + move arm 45 deg to assign slot
    Step 4 | Rename CAN ports and save config
    """

    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("Piper Multi-Arm Setup")
        self.root.resizable(True, True)

        self._arms: list[DiscoveredArm] = []
        self._config_var = tk.StringVar(value=list(CONFIGS.keys())[0])
        self._bitrate_var = tk.StringVar(value="1000000")

        # Step 3 state
        self._slot_rows: list[dict] = []
        self._find_active = False
        self._find_cancelled = False

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)


    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)
        # only the log panel (row 4) expands vertically
        for r, w in [(0, 0), (1, 0), (2, 0), (3, 0), (4, 1)]:
            self.root.rowconfigure(r, weight=w)

        self._build_step1(row=0)
        self._build_step2(row=1)
        self._build_step3(row=2)
        self._build_step4(row=3)
        self._build_log(row=4)

    # ── Step 1 ─────────────────────────────────────────────────────────────

    def _build_step1(self, row: int) -> None:
        frm = ttk.LabelFrame(
            self.root, text="Step 1 — Scan CAN ports and verify Piper arms", padding=6)
        frm.grid(row=row, column=0, sticky="ew", padx=8, pady=(8, 4))
        frm.columnconfigure(0, weight=1)

        toolbar = ttk.Frame(frm)
        toolbar.pack(fill="x", pady=(0, 6))
        ttk.Label(toolbar, text="Bitrate:").pack(side="left")
        ttk.Entry(toolbar, textvariable=self._bitrate_var, width=10).pack(side="left", padx=4)
        self._scan_btn = ttk.Button(toolbar, text="Scan & Connect", command=self._on_scan)
        self._scan_btn.pack(side="left", padx=4)
        self._scan_status_var = tk.StringVar(value="")
        ttk.Label(toolbar, textvariable=self._scan_status_var,
                  foreground="#555555").pack(side="left", padx=8)

        cols = ("CAN Port", "USB Port (bus-info)", "Firmware", "ctrl_mode")
        self._scan_tree = ttk.Treeview(frm, columns=cols, show="headings", height=5)
        widths = {"CAN Port": 105, "USB Port (bus-info)": 195, "Firmware": 110, "ctrl_mode": 150}
        for c in cols:
            self._scan_tree.heading(c, text=c)
            self._scan_tree.column(c, width=widths[c], stretch=(c == "USB Port (bus-info)"))
        self._scan_tree.tag_configure("piper",   foreground="#005500")
        self._scan_tree.tag_configure("none",    foreground="#aaaaaa")
        self._scan_tree.pack(fill="x")

    # ── Step 2 ─────────────────────────────────────────────────────────────

    def _build_step2(self, row: int) -> None:
        frm = ttk.LabelFrame(self.root, text="Step 2 — Select Configuration", padding=6)
        frm.grid(row=row, column=0, sticky="ew", padx=8, pady=4)

        rb_frm = ttk.Frame(frm)
        rb_frm.pack(fill="x")
        for key in CONFIGS:
            ttk.Radiobutton(rb_frm, text=key, variable=self._config_var,
                            value=key).pack(side="left", padx=12)

        self._apply_config_btn = ttk.Button(
            frm, text="Apply Config",
            command=self._on_apply_config,
            state="disabled")
        self._apply_config_btn.pack(anchor="e", pady=(6, 0))

    # ── Step 3 ─────────────────────────────────────────────────────────────

    def _build_step3(self, row: int) -> None:
        frm = ttk.LabelFrame(
            self.root,
            text="Step 3 — Identify Arms  (click Find, then move the arm more than 45 deg)",
            padding=6)
        frm.grid(row=row, column=0, sticky="ew", padx=8, pady=4)
        frm.columnconfigure(0, weight=1)

        self._step3_placeholder = ttk.Label(
            frm,
            text="Select a configuration in Step 2 and click 'Apply Config'.",
            foreground="#888888")
        self._step3_placeholder.pack(pady=10)

        self._step3_table = ttk.Frame(frm)
        self._step3_table.pack(fill="x")

    def _rebuild_step3_table(self) -> None:
        """Rebuild the slot table for the selected configuration."""
        for w in self._step3_table.winfo_children():
            w.destroy()
        self._slot_rows.clear()

        slots = CONFIGS[self._config_var.get()]
        t = self._step3_table
        t.columnconfigure(2, weight=1)

        for c, h in enumerate(["Slot", "New CAN Name", "Assigned Port", "Status", "", "", ""]):
            ttk.Label(t, text=h).grid(
                row=0, column=c, sticky="w", padx=6, pady=(0, 2))
        ttk.Separator(t, orient="horizontal").grid(
            row=1, column=0, columnspan=7, sticky="ew", pady=2)

        for i, slot in enumerate(slots):
            r = i + 2
            status_var  = tk.StringVar(value="Waiting")
            assigned_var = tk.StringVar(value="(none)")

            ttk.Label(t, text=slot_to_label(slot), width=13).grid(
                row=r, column=0, sticky="w", padx=6, pady=4)
            ttk.Label(t, text=slot_to_can_name(slot), width=15,
                      foreground="#005599").grid(row=r, column=1, sticky="w", padx=6)
            ttk.Label(t, textvariable=assigned_var, width=14).grid(
                row=r, column=2, sticky="w", padx=6)
            ttk.Label(t, textvariable=status_var, width=18).grid(
                row=r, column=3, sticky="w", padx=6)
            btn = ttk.Button(t, text="Find", width=8,
                             command=lambda s=slot: self._on_find(s))
            btn.grid(row=r, column=4, padx=6)

            btn_role = ttk.Button(t, text="Set Role", width=9,
                                  state="disabled",
                                  command=lambda s=slot: self._on_set_role(s))
            btn_role.grid(row=r, column=5, padx=6)

            btn_torque_off = ttk.Button(t, text="Torque Off", width=10,
                                        state="disabled",
                                        command=lambda s=slot: self._on_torque_off(s))
            btn_torque_off.grid(row=r, column=6, padx=6)

            self._slot_rows.append({
                "slot":            slot,
                "new_name":        slot_to_can_name(slot),
                "label":           slot_to_label(slot),
                "status_var":      status_var,
                "assigned_var":    assigned_var,
                "btn":             btn,
                "btn_role":        btn_role,
                "btn_torque_off":  btn_torque_off,
            })

    # ── Step 4 ─────────────────────────────────────────────────────────────

    def _build_step4(self, row: int) -> None:
        frm = ttk.LabelFrame(
            self.root,
            text="Step 4 — Finalize: rename CAN ports and save config",
            padding=6)
        frm.grid(row=row, column=0, sticky="ew", padx=8, pady=4)

        self._finalize_btn = ttk.Button(
            frm, text="Save Config & Reinitialize CAN Ports",
            command=self._on_finalize,
            state="disabled")
        self._finalize_btn.pack(side="left", padx=4)
        self._finalize_status_var = tk.StringVar(value="")
        ttk.Label(frm, textvariable=self._finalize_status_var).pack(
            side="left", padx=8)

    def _build_log(self, row: int) -> None:
        frm = ttk.LabelFrame(self.root, text="Log", padding=4)
        frm.grid(row=row, column=0, sticky="nsew", padx=8, pady=(4, 8))
        frm.columnconfigure(0, weight=1)
        frm.rowconfigure(0, weight=1)

        self._log_box = scrolledtext.ScrolledText(
            frm, height=9, state="disabled",
            wrap="word")
        self._log_box.grid(row=0, column=0, sticky="nsew")
        ttk.Button(frm, text="Clear", command=self._log_clear).grid(
            row=1, column=0, sticky="e", pady=2)

    # ── Step 1 ────────────────────────────────────────────────────────────

    def _on_scan(self) -> None:
        self._scan_btn.config(state="disabled")
        self._apply_config_btn.config(state="disabled")
        self._scan_status_var.set("Scanning...")

        # disconnect existing connections
        for arm in self._arms:
            arm.disconnect()
        self._arms.clear()
        for item in self._scan_tree.get_children():
            self._scan_tree.delete(item)

        bitrate = self._parse_bitrate()
        threading.Thread(target=self._scan_worker, args=(bitrate,),
                         daemon=True).start()

    def _scan_worker(self, bitrate: int) -> None:
        interfaces = detect_can_interfaces()
        if not interfaces:
            self._log("No CAN ports found. Check that USB-CAN adapters are connected.",
                      level="warn")
            self._ui(lambda: self._scan_status_var.set("No CAN ports"))
            self._ui(lambda: self._scan_btn.config(state="normal"))
            return

        self._log(f"Found {len(interfaces)} CAN port(s). Checking each for Piper...")

        for d in interfaces:
            iface, bus_info = d["iface"], d["bus_info"]
            self._log(f"  {iface}: initializing and connecting...")
            arm = DiscoveredArm(iface, bus_info)
            is_piper = arm.connect_and_verify(bitrate)

            if is_piper:
                self._log(f"  {iface}: Piper detected  fw={arm.fw}  ctrl_mode={arm.ctrl_mode_str}")
                self._ui(lambda a=arm:
                         self._scan_tree.insert("", "end",
                             values=(a.iface, a.bus_info, a.fw, a.ctrl_mode_str),
                             tags=("piper",)))
                self._arms.append(arm)
            else:
                self._log(f"  {iface}: no Piper or no response", level="warn")
                self._ui(lambda i=iface, b=bus_info:
                         self._scan_tree.insert("", "end",
                             values=(i, b, "-", "none"),
                             tags=("none",)))

        n = len(self._arms)
        self._ui(lambda: self._scan_status_var.set(f"{n} Piper arm(s) found"))
        self._ui(lambda: self._scan_btn.config(state="normal"))
        if n > 0:
            self._ui(lambda: self._apply_config_btn.config(state="normal"))
            self._log(f"Scan complete — {n} Piper arm(s). Select a configuration in Step 2.")
        else:
            self._log("No Piper arms found.", level="warn")

    # ── Step 2 ────────────────────────────────────────────────────────────

    def _on_apply_config(self) -> None:
        config_name = self._config_var.get()
        required = len(CONFIGS[config_name])
        if len(self._arms) < required:
            messagebox.showwarning(
                "Not enough arms",
                f"'{config_name}' requires {required} Piper arm(s) but "
                f"only {len(self._arms)} were detected.\n\n"
                "Re-run Step 1 or choose a different configuration."
            )
            return

        for arm in self._arms:
            arm.slot = None
            arm.new_name = None

        self._step3_placeholder.pack_forget()
        self._rebuild_step3_table()
        self._finalize_btn.config(state="disabled")
        self._finalize_status_var.set("")
        self._log(f"Config '{config_name}' applied. "
                  f"Click [Find] for each slot and move the arm more than 45 deg.")

    # ── Step 3 ────────────────────────────────────────────────────────────

    def _on_find(self, slot: str) -> None:
        # While find is active the button acts as a cancel button
        if self._find_active:
            self._find_cancelled = True
            return

        row_data = next((r for r in self._slot_rows if r["slot"] == slot), None)
        if row_data is None:
            return

        # exclude already-assigned arms
        unassigned = [a for a in self._arms if a.slot is None]
        if not unassigned:
            messagebox.showinfo("Done", "All arms have been assigned.")
            return

        self._find_active = True
        self._find_cancelled = False
        row_data["status_var"].set(f"Finding... ({FIND_TIMEOUT_SEC}s)")
        row_data["btn"].config(state="normal", text="■ Cancel")
        self._log(f"{row_data['label']}: finding started "
                  f"— move the arm more than 45 deg within {FIND_TIMEOUT_SEC}s.")

        threading.Thread(
            target=self._find_worker,
            args=(row_data, unassigned),
            daemon=True
        ).start()

    def _find_worker(self, row_data: dict, candidates: list[DiscoveredArm]) -> None:
        """Monitor unassigned arms and assign the one that moves more than 45 deg."""
        slot  = row_data["slot"]
        label = row_data["label"]
        new_name = row_data["new_name"]

        # Wait for CAN messages to stabilize
        time.sleep(0.3)

        # Record baselines + diagnostic log
        baselines: dict[str, list[int]] = {}
        for arm in candidates:
            pos = arm.read_joints_raw()
            if pos:
                baselines[arm.iface] = pos
                self._log(f"  [{arm.iface}/{arm.role}] baseline J1-6: "
                          f"{[round(v/1000,1) for v in pos]} deg")
            else:
                self._log(f"  [{arm.iface}/{arm.role}] baseline read failed — None returned",
                          level="warn")

        deadline = time.time() + FIND_TIMEOUT_SEC
        found_arm: DiscoveredArm | None = None

        while time.time() < deadline and not found_arm and not self._find_cancelled:
            time.sleep(0.05)
            remaining = max(0, int(deadline - time.time()))
            self._ui(lambda r=remaining:
                     row_data["status_var"].set(f"Finding... ({r}s)"))

            for arm in candidates:
                if arm.iface not in baselines:
                    continue
                pos = arm.read_joints_raw()
                if pos is None:
                    continue
                base = baselines[arm.iface]
                max_delta = max(abs(pos[i] - base[i]) for i in range(6))
                if max_delta >= FIND_THRESHOLD_RAW:
                    found_arm = arm
                    break

        self._find_active = False

        if found_arm:
            found_arm.slot = slot
            found_arm.new_name = new_name
            found_arm.role = slot.rsplit("_", 1)[0]  # "leader" or "follower" from slot name
            iface = found_arm.iface
            self._log(f"  + {label} assigned: {iface}  ->  {new_name}")
            self._ui(lambda: row_data["status_var"].set("Assigned"))
            self._ui(lambda: row_data["assigned_var"].set(iface))
            self._ui(lambda: row_data["btn"].config(state="disabled", text="Done"))
            self._ui(lambda: row_data["btn_role"].config(state="normal"))
            self._ui(lambda: row_data["btn_torque_off"].config(state="normal"))

            all_done = all(
                any(a.slot == r["slot"] for a in self._arms)
                for r in self._slot_rows
            )
            if all_done:
                self._ui(lambda: self._finalize_btn.config(state="normal"))
                self._log("All slots assigned! Proceed to Step 4.")
        elif self._find_cancelled:
            self._log(f"  - {label}: find cancelled")
            self._ui(lambda: row_data["status_var"].set("Cancelled"))
            self._ui(lambda: row_data["btn"].config(state="normal", text="Find"))
        else:
            self._log(f"  x {label}: timeout ({FIND_TIMEOUT_SEC}s)", level="warn")
            self._ui(lambda: row_data["status_var"].set("Timeout"))
            self._ui(lambda: row_data["btn"].config(state="normal", text="Find"))

    def _on_set_role(self, slot: str) -> None:
        """Apply MasterSlaveConfig to the arm assigned to this slot."""
        arm = next((a for a in self._arms if a.slot == slot), None)
        if arm is None:
            return
        row_data = next((r for r in self._slot_rows if r["slot"] == slot), None)

        def _worker() -> None:
            config = 0xFA if arm.role == "leader" else 0xFC
            label = "leader (0xFA)" if arm.role == "leader" else "follower (0xFC)"
            try:
                with arm._lock:
                    if arm.piper is None:
                        self._log(f"  {arm.iface}: not connected", level="warn")
                        return
                    arm.piper.MasterSlaveConfig(config, 0, 0, 0)
                self._log(f"  {arm.iface}: MasterSlaveConfig set -> {label}")
                if row_data:
                    self._ui(lambda: row_data["status_var"].set(f"Role set ({arm.role})"))
            except Exception as e:
                self._log(f"  {arm.iface}: MasterSlaveConfig failed: {e}", level="error")

        threading.Thread(target=_worker, daemon=True).start()

    def _on_torque_off(self, slot: str) -> None:
        """Disable torque on the arm assigned to this slot."""
        arm = next((a for a in self._arms if a.slot == slot), None)
        if arm is None:
            return

        def _worker() -> None:
            try:
                with arm._lock:
                    if arm.piper is None:
                        self._log(f"  {arm.iface}: not connected", level="warn")
                        return
                    arm.piper.DisablePiper()
                self._log(f"  {arm.iface}: torque disabled")
            except Exception as e:
                self._log(f"  {arm.iface}: torque off failed: {e}", level="error")

        threading.Thread(target=_worker, daemon=True).start()

    # ── Step 4 ────────────────────────────────────────────────────────────

    def _on_finalize(self) -> None:
        assigned = [(a.iface, a.new_name) for a in self._arms
                    if a.slot and a.new_name]
        if not assigned:
            return

        lines = "\n".join(f"  {old}  ->  {new}" for old, new in assigned)
        if not messagebox.askyesno(
            "Reinitialize CAN ports",
            f"The following CAN ports will be renamed and reinitialized:\n\n{lines}\n\nContinue?"
        ):
            return

        self._finalize_btn.config(state="disabled")
        threading.Thread(target=self._finalize_worker, args=(assigned,),
                         daemon=True).start()

    def _finalize_worker(self, assignments: list[tuple[str, str]]) -> None:
        # Apply MasterSlaveConfig before disconnecting (while SDK connections are live)
        for arm in self._arms:
            if arm.slot is None:
                continue
            with arm._lock:
                if arm.piper is None:
                    continue
                try:
                    if arm.role == "leader":
                        arm.piper.MasterSlaveConfig(0xFA, 0, 0, 0)
                        self._log(f"  {arm.iface}: MasterSlaveConfig -> leader (0xFA)")
                    else:
                        arm.piper.MasterSlaveConfig(0xFC, 0, 0, 0)
                        self._log(f"  {arm.iface}: MasterSlaveConfig -> follower (0xFC)")
                except Exception as e:
                    self._log(f"  {arm.iface}: MasterSlaveConfig failed: {e}", level="warn")
        time.sleep(0.3)

        # Disconnect all SDK connections before renaming ports
        for arm in self._arms:
            arm.disconnect()
        self._log("All connections closed.")

        success_all = True
        for old_name, new_name in assignments:
            self._log(f"  {old_name}  ->  {new_name}  renaming...")
            ok, msg = rename_can_interface(old_name, new_name)
            if ok:
                self._log(f"  + {old_name}  ->  {new_name}  done")
            else:
                self._log(f"  x {old_name} rename failed: {msg}", level="error")
                success_all = False

        if success_all:
            save_path = self._save_config()
            self._ui(lambda p=save_path: self._finalize_status_var.set(
                f"Done! Config saved: {p}"))
            self._log(f"Config saved: {save_path}")
            self._log("Done! You can now start teleoperation with the new port names.")
        else:
            self._log("Some operations failed. Check the log and try again.",
                      level="error")
            self._ui(lambda: self._finalize_btn.config(state="normal"))

    def _save_config(self) -> str:
        """Save assignment results to ~/piper_config.json."""
        save_path = os.path.expanduser("~/piper_config.json")
        data = {
            "configuration": self._config_var.get(),
            "arms": [
                {
                    "slot":            a.slot,
                    "can_name":        a.new_name,
                    "original_iface":  a.iface,
                    "bus_info":        a.bus_info,
                    "role":            a.role,
                    "firmware":        a.fw,
                }
                for a in self._arms if a.slot
            ],
        }
        with open(save_path, "w") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        return save_path


    def _parse_bitrate(self) -> int:
        try:
            return int(self._bitrate_var.get())
        except ValueError:
            return 1_000_000

    def _ui(self, fn) -> None:
        """Thread-safe UI update from a background thread."""
        self.root.after(0, fn)

    def _log(self, msg: str, level: str = "info") -> None:
        ts = time.strftime("%H:%M:%S")
        prefix = {"info": "", "warn": "[WARN] ", "error": "[ERROR] "}.get(level, "")
        line = f"[{ts}] {prefix}{msg}\n"

        def _append():
            self._log_box.config(state="normal")
            self._log_box.insert("end", line)
            self._log_box.see("end")
            self._log_box.config(state="disabled")

        self._ui(_append)

    def _log_clear(self) -> None:
        self._log_box.config(state="normal")
        self._log_box.delete("1.0", "end")
        self._log_box.config(state="disabled")

    def _on_close(self) -> None:
        self._find_active = False
        for arm in self._arms:
            arm.disconnect()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


# ─────────────────────────── entry point ─────────────────────────────────


def main() -> None:
    app = ArmSetupUI()
    app.run()


if __name__ == "__main__":
    main()
