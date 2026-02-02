#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import json
import math
import os
import threading
import time
import tkinter as tk
from tkinter import messagebox

import ttkbootstrap as ttk
from ttkbootstrap.constants import *
from ttkbootstrap.widgets.scrolled import ScrolledText
import mainControl as mc


class RobotDashboard(ttk.Window):
    def __init__(self):
        super().__init__(themename="cyborg")

        self.title("Robot Control Dashboard (MQTT)")
        self.geometry("900x650")
        self.minsize(850, 600)

        self._connected = False
        self._pose_poll_stop = threading.Event()
        self._pose_thread = None
        self._busy_lock = threading.Lock()
        self._pose_log_path = self._init_pose_log()
        self._pose_log_interval_s = 1.0
        self._last_pose_log_ts = 0.0
        self._pose_logging_enabled = False

        self._last_status = None
        self._last_pose_ok_ts = 0.0
        self._last_pose_err_log_ts = 0.0
        self._last_error_sig = None

        self._build_ui()
        self._bind_hotkeys()

        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self):
        main_container = ttk.Frame(self, padding=20)
        main_container.pack(fill="both", expand=True)

        header_frame = ttk.Frame(main_container)
        header_frame.pack(fill="x", pady=(0, 20))

        self.conn_btn = ttk.Button(
            header_frame,
            text="🔌 CONNECT",
            command=self.toggle_connect,
            bootstyle="success-outline",
            width=15
        )
        self.conn_btn.pack(side="left")

        self.reset_btn = ttk.Button(
            header_frame,
            text="♻ Reset Session",
            command=self.reset_session,
            state="disabled",
            bootstyle="warning-outline"
        )
        self.reset_btn.pack(side="left", padx=10)

        self.btn_log_start = ttk.Button(
            header_frame,
            text="▶ Start Log",
            command=self.start_pose_logging,
            bootstyle="secondary-outline",
            width=12
        )
        self.btn_log_start.pack(side="left", padx=(0, 5))

        self.btn_log_stop = ttk.Button(
            header_frame,
            text="■ Stop Log",
            command=self.stop_pose_logging,
            bootstyle="secondary",
            width=12
        )
        self.btn_log_stop.pack(side="left")

        self.status_var = tk.StringVar(value="DISCONNECTED")
        self.lbl_status = ttk.Label(
            header_frame,
            textvariable=self.status_var,
            font=("Helvetica", 10, "bold"),
            bootstyle="danger"
        )
        self.lbl_status.pack(side="right", padx=10)
        ttk.Label(header_frame, text="STATUS:").pack(side="right")

        content_frame = ttk.Frame(main_container)
        content_frame.pack(fill="both", expand=True)

        left_col = ttk.Frame(content_frame)
        left_col.pack(side="left", fill="y", expand=False, padx=(0, 20))

        pose_group = ttk.Labelframe(left_col, text="📍 Robot Pose", padding=15, bootstyle="info")
        pose_group.pack(fill="x", pady=(0, 15))

        self.pose_x = tk.StringVar(value="0.000")
        self.pose_y = tk.StringVar(value="0.000")
        self.pose_theta = tk.StringVar(value="0.0")

        self._create_stat_card(pose_group, "X (m)", self.pose_x, 0, 0)
        self._create_stat_card(pose_group, "Y (m)", self.pose_y, 0, 1)
        self._create_stat_card(pose_group, "Theta (deg)", self.pose_theta, 0, 2)

        map_group = ttk.Labelframe(left_col, text="🗺 Map Info", padding=15, bootstyle="secondary")
        map_group.pack(fill="x")

        self.pose_map = tk.StringVar(value="-")
        self.pose_desc = tk.StringVar(value="-")

        ttk.Label(map_group, text="ID:", foreground="gray").grid(row=0, column=0, sticky="w")
        ttk.Label(map_group, textvariable=self.pose_map, font=("Consolas", 11, "bold")).grid(row=0, column=1, sticky="w", padx=10)

        ttk.Label(map_group, text="Desc:", foreground="gray").grid(row=1, column=0, sticky="w", pady=(5, 0))
        ttk.Label(map_group, textvariable=self.pose_desc).grid(row=1, column=1, sticky="w", padx=10, pady=(5, 0))

        right_col = ttk.Frame(content_frame)
        right_col.pack(side="left", fill="both", expand=True)

        ctl_notebook = ttk.Notebook(right_col)
        ctl_notebook.pack(fill="both", expand=True)

        manual_tab = ttk.Frame(ctl_notebook, padding=15)
        ctl_notebook.add(manual_tab, text="🎮 Manual Control")

        self.btn_stop = ttk.Button(
            manual_tab,
            text="🛑 EMERGENCY STOP",
            command=self.emergency_stop,
            bootstyle="danger", 
            width=20
        )
        self.btn_stop.pack(fill="x", pady=(0, 15), ipady=5)
        # ------------------------------

        ctrl_grid = ttk.Frame(manual_tab)
        ctrl_grid.pack(fill="both", expand=True)

        move_frame = ttk.Labelframe(ctrl_grid, text="Movement", padding=15, bootstyle="primary")
        move_frame.pack(side="left", fill="both", expand=True, padx=(0, 10))

        step_frame = ttk.Frame(move_frame)
        step_frame.pack(fill="x", pady=(0, 15))
        ttk.Label(step_frame, text="Step (m):").pack(side="left")
        self.move_step = tk.DoubleVar(value=1)
        ttk.Entry(step_frame, textvariable=self.move_step, width=8).pack(side="left", padx=5)

        dpad = ttk.Frame(move_frame)
        dpad.pack(expand=True)

        self.btn_front = ttk.Button(dpad, text="▲", command=lambda: self._run_cmd(self.move_front), bootstyle="primary")
        self.btn_back = ttk.Button(dpad, text="▼", command=lambda: self._run_cmd(self.move_back), bootstyle="primary")
        self.btn_left = ttk.Button(dpad, text="◀", command=lambda: self._run_cmd(self.move_left), bootstyle="primary")
        self.btn_right = ttk.Button(dpad, text="▶", command=lambda: self._run_cmd(self.move_right), bootstyle="primary")

        self.btn_front.grid(row=0, column=1, padx=5, pady=5, ipadx=10, ipady=10)
        self.btn_left.grid(row=1, column=0, padx=5, pady=5, ipadx=10, ipady=10)
        self.btn_back.grid(row=1, column=1, padx=5, pady=5, ipadx=10, ipady=10)
        self.btn_right.grid(row=1, column=2, padx=5, pady=5, ipadx=10, ipady=10)

        ttk.Label(move_frame, text="Arrows", font=("Arial", 8), foreground="gray").pack(side="bottom", pady=5)

        rot_frame = ttk.Labelframe(ctrl_grid, text="Rotation", padding=15, bootstyle="warning")
        rot_frame.pack(side="left", fill="both", expand=True)

        rot_step_frame = ttk.Frame(rot_frame)
        rot_step_frame.pack(fill="x", pady=(0, 15))
        ttk.Label(rot_step_frame, text="Angle (°):").pack(side="left")
        self.rotate_step = tk.DoubleVar(value=90.0)
        ttk.Entry(rot_step_frame, textvariable=self.rotate_step, width=8).pack(side="left", padx=5)

        rot_btns = ttk.Frame(rot_frame)
        rot_btns.pack(expand=True)

        self.btn_ccw = ttk.Button(rot_btns, text="↺ CCW", command=lambda: self._run_cmd(self.rotate_ccw), bootstyle="warning", width=10)
        self.btn_cw = ttk.Button(rot_btns, text="CW ↻", command=lambda: self._run_cmd(self.rotate_cw), bootstyle="warning", width=10)

        self.btn_ccw.pack(pady=5, ipadx=5, ipady=5)
        self.btn_cw.pack(pady=5, ipadx=5, ipady=5)

        log_group = ttk.Labelframe(main_container, text="System Log", padding=10)
        log_group.pack(fill="both", expand=True, pady=(15, 0))

        self.log = ScrolledText(log_group, height=6, autohide=True, font=("Consolas", 9))
        self.log.pack(fill="both", expand=True)

        self._update_log_buttons()
        self._log_line("Ready. Click Connect to start.")
        self._log_line(f"Pose log file: {self._pose_log_path}")
        self._set_controls_state(False)

    def _init_pose_log(self) -> str:
        path = os.path.join(os.path.dirname(__file__), "robot_pose.log.tsv")
        if not os.path.exists(path):
            with open(path, "w", encoding="utf-8", newline="") as f:
                writer = csv.writer(f, delimiter="\t")
                writer.writerow(
                    ["timestamp", "x", "y", "theta_rad", "theta_deg", "map_id", "map_description"]
                )
        return path

    def _append_pose_log(self, pose):
        if not self._pose_logging_enabled:
            return
        now = time.time()
        if now - self._last_pose_log_ts < self._pose_log_interval_s:
            return
        self._last_pose_log_ts = now
        ts = time.strftime("%Y-%m-%d %H:%M:%S")
        with open(self._pose_log_path, "a", encoding="utf-8", newline="") as f:
            writer = csv.writer(f, delimiter="\t")
            writer.writerow(
                [
                    ts,
                    f"{pose.x:.3f}",
                    f"{pose.y:.3f}",
                    f"{pose.theta:.6f}",
                    f"{math.degrees(pose.theta):.3f}",
                    pose.map_id,
                    pose.map_description,
                ]
            )

    def _create_stat_card(self, parent, title, var, row, col):
        frame = ttk.Frame(parent, padding=5)
        frame.grid(row=row, column=col, sticky="ns", padx=5)
        ttk.Label(frame, text=title, font=("Arial", 8), foreground="gray").pack()
        ttk.Label(frame, textvariable=var, font=("Consolas", 14, "bold"), bootstyle="info").pack()

    def _bind_hotkeys(self):
        self.bind("<w>", lambda e: self._run_cmd(self.move_front))
        self.bind("<s>", lambda e: self._run_cmd(self.move_back))
        self.bind("<a>", lambda e: self._run_cmd(self.move_left))
        self.bind("<d>", lambda e: self._run_cmd(self.move_right))

        self.bind("<q>", lambda e: self._run_cmd(self.rotate_ccw))
        self.bind("<e>", lambda e: self._run_cmd(self.rotate_cw))

        self.bind("<Up>", lambda e: self._run_cmd(self.move_front))
        self.bind("<Down>", lambda e: self._run_cmd(self.move_back))
        self.bind("<Left>", lambda e: self._run_cmd(self.move_left))
        self.bind("<Right>", lambda e: self._run_cmd(self.move_right))

        self.bind("<space>", lambda e: self.emergency_stop())

    def _set_controls_state(self, enabled: bool):
        state = "normal" if enabled else "disabled"
        for b in [self.reset_btn, self.btn_front, self.btn_back, self.btn_left, self.btn_right, self.btn_cw, self.btn_ccw, self.btn_stop]:
            b.config(state=state)

    def _log_line(self, s: str):
        ts = time.strftime("%H:%M:%S")
        self.log.insert("end", f"[{ts}] {s}\n")
        self.log.see("end")

    def _set_status(self, text: str, style: str):
        if text == self._last_status:
            return
        self._last_status = text
        self.status_var.set(text)
        self.lbl_status.config(bootstyle=style)

    def _log_state_errors(self, state):
        errors = state.get("errors", [])
        if not isinstance(errors, list):
            return

        if not errors:
            if self._last_error_sig is not None:
                self._last_error_sig = None
                self._log_line("Errors cleared.")
            return

        sig = json.dumps(errors, ensure_ascii=True, sort_keys=True)
        if sig == self._last_error_sig:
            return
        self._last_error_sig = sig

        self._log_line(f"Robot errors: {len(errors)}")
        for err in errors:
            if not isinstance(err, dict):
                continue
            err_type = str(err.get("errorType") or "Unknown")
            level = str(err.get("errorLevel") or "UNKNOWN")
            desc = str(err.get("errorDescription") or "").strip()
            refs = err.get("errorReferences", [])
            ref_parts = []
            if isinstance(refs, list):
                for ref in refs:
                    if isinstance(ref, dict):
                        k = str(ref.get("referenceKey") or "").strip()
                        v = str(ref.get("referenceValue") or "").strip()
                        if k or v:
                            ref_parts.append(f"{k}={v}" if k else v)
            ref_str = f" ({', '.join(ref_parts)})" if ref_parts else ""
            msg = f"ERROR[{level}] {err_type}"
            if desc:
                msg = f"{msg}: {desc}"
            self._log_line(f"{msg}{ref_str}")

    def toggle_connect(self):
        if not self._connected:
            self.connect()
        else:
            self.disconnect()

    def connect(self):
        try:
            mc.reset_order_session()
            mc._ensure_client_connected()

            self._connected = True
            self.conn_btn.config(text="DISCONNECT", bootstyle="danger-outline")
            self._set_status("CONNECTED", "success")
            self._set_controls_state(True)
            self._log_line("Connected. Subscribed to state topic.")

            self._pose_poll_stop.clear()
            self._pose_thread = threading.Thread(target=self._pose_poll_loop, daemon=True)
            self._pose_thread.start()

        except Exception as ex:
            self._connected = False
            self.conn_btn.config(text="🔌 CONNECT", bootstyle="success-outline")
            self._set_controls_state(False)
            self._set_status("DISCONNECTED", "danger")

            messagebox.showerror("Connect failed", str(ex))
            self._log_line(f"Connect failed: {ex}")

    def disconnect(self):
        try:
            self._pose_poll_stop.set()
            mc.disconnect()
        except Exception as ex:
            self._log_line(f"Disconnect error: {ex}")
        finally:
            self._connected = False
            self.conn_btn.config(text="🔌 CONNECT", bootstyle="success-outline")
            self._set_controls_state(False)
            self._set_status("DISCONNECTED", "danger")
            self._log_line("Disconnected.")

    def _pose_poll_loop(self):
        while not self._pose_poll_stop.is_set():
            if not self._connected:
                break
            try:
                state = mc._get_latest_state(timeout_s=2.0)
                pose = mc._try_parse_pose_from_state(state)
                if pose is not None:
                    self._last_pose_ok_ts = time.time()
                    self.after(0, lambda p=pose: self._update_pose_ui(p))
                else:
                    self.after(0, lambda: self._set_status("WAITING_POSE", "warning"))

                self.after(0, lambda s=state: self._log_state_errors(s))
            except Exception:
                self.after(0, lambda: self._set_status("WAITING_STATE", "warning"))

                now = time.time()
                if now - self._last_pose_err_log_ts >= 5.0:
                    self._last_pose_err_log_ts = now
                    self.after(0, lambda: self._log_line("No state received yet (waiting state topic)..."))
            time.sleep(0.25)

    def _update_pose_ui(self, pose):
        self.pose_x.set(f"{pose.x:.3f}")
        self.pose_y.set(f"{pose.y:.3f}")
        self.pose_theta.set(f"{math.degrees(pose.theta):.1f}")
        self.pose_map.set(f"{pose.map_id}")
        self.pose_desc.set(f"{pose.map_description}")
        self._append_pose_log(pose)

        self._set_status("ONLINE", "success")

    def reset_session(self):
        try:
            mc.reset_order_session()
            self._log_line("Order session reset (new orderId).")
        except Exception as ex:
            self._log_line(f"Reset error: {ex}")

    def _update_log_buttons(self):
        if self._pose_logging_enabled:
            self.btn_log_start.config(state="disabled")
            self.btn_log_stop.config(state="normal")
        else:
            self.btn_log_start.config(state="normal")
            self.btn_log_stop.config(state="disabled")

    def start_pose_logging(self):
        if self._pose_logging_enabled:
            return
        self._pose_logging_enabled = True
        self._last_pose_log_ts = 0.0
        self._update_log_buttons()
        self._log_line("Pose logging started.")

    def stop_pose_logging(self):
        if not self._pose_logging_enabled:
            return
        self._pose_logging_enabled = False
        self._update_log_buttons()
        self._log_line("Pose logging stopped.")

    def _set_manual_controls_temporarily(self, enabled: bool):
        state = "normal" if enabled else "disabled"
        for b in [self.reset_btn, self.btn_front, self.btn_back, self.btn_left, self.btn_right, self.btn_cw, self.btn_ccw]:
            b.config(state=state)

    def _run_cmd(self, fn):
        if not self._connected:
            self._log_line("Not connected.")
            return

        if not self._busy_lock.acquire(blocking=False):
            self._log_line("Busy...")
            return

        self.after(0, lambda: self._set_manual_controls_temporarily(False))

        def worker():
            try:
                fn()
            except Exception as ex:
                self.after(0, lambda: self._log_line(f"Cmd error: {ex}"))
            finally:
                self._busy_lock.release()
                self.after(0, lambda: self._set_manual_controls_temporarily(self._connected))

        threading.Thread(target=worker, daemon=True).start()

    def emergency_stop(self):
        if not self._connected:
            return
        self._log_line("!!! EMERGENCY STOP !!!")
        def worker():
            try:
                if hasattr(mc, "stop_robot"):
                    mc.stop_robot()
                elif hasattr(mc, "send_cancel_order"):
                    mc.send_cancel_order()
                else:
                    mc.manualMove_front(0)
                self.after(0, lambda: self._log_line("Stop sent."))
            except Exception as ex:
                self.after(0, lambda: self._log_line(f"Stop error: {ex}"))

        threading.Thread(target=worker, daemon=True).start()

    def _get_move_val(self) -> float:
        try:
            return float(self.move_step.get())
        except Exception:
            return 0.0

    def _get_rot_val(self) -> float:
        try:
            return float(self.rotate_step.get())
        except Exception:
            return 0.0

    def move_front(self):
        d = self._get_move_val()
        self._log_line(f"Move front: {d} m")
        mc.manualMove_front(d)

    def move_back(self):
        d = self._get_move_val()
        self._log_line(f"Move back: {d} m")
        mc.manualMove_back(d)

    def move_left(self):
        d = self._get_move_val()
        self._log_line(f"Move left: {d} m")
        mc.manualMove_left(d)

    def move_right(self):
        d = self._get_move_val()
        self._log_line(f"Move right: {d} m")
        mc.manualMove_right(d)

    def rotate_cw(self):
        a = self._get_rot_val()
        self._log_line(f"Rotate CW: {a} deg")
        mc.manualRotate_clockwise(a)

    def rotate_ccw(self):
        a = self._get_rot_val()
        self._log_line(f"Rotate CCW: {a} deg")
        mc.manualRotate_counterclockwise(a)

    def on_close(self):
        try:
            if self._connected:
                self.disconnect()
        finally:
            self.destroy()

if __name__ == "__main__":
    app = RobotDashboard()
    app.place_window_center()
    app.mainloop()
