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
        self._trail_points = []
        self._trail_max_points = 1500

        # Go-to coordinate inputs
        self.goto_x = tk.StringVar(value="0.0")
        self.goto_y = tk.StringVar(value="0.0")
        self.goto_theta = tk.StringVar(value="")  # radians (optional)

        self._build_ui()
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

        viz_group = ttk.Labelframe(left_col, text="🧭 Trajectory", padding=10, bootstyle="dark")
        viz_group.pack(fill="both", expand=True, pady=(15, 0))

        self.trail_canvas = tk.Canvas(viz_group, background="#101820", highlightthickness=0, height=220)
        self.trail_canvas.pack(fill="both", expand=True)
        self.trail_canvas.bind("<Configure>", lambda _evt: self._redraw_trail())

        viz_btn_row = ttk.Frame(viz_group)
        viz_btn_row.pack(fill="x", pady=(8, 0))
        self.btn_clear_trail = ttk.Button(
            viz_btn_row,
            text="Clear Trail",
            command=self.clear_trail,
            bootstyle="secondary-outline",
            width=12,
        )
        self.btn_clear_trail.pack(side="right")

        right_col = ttk.Frame(content_frame)
        right_col.pack(side="left", fill="both", expand=True)

        ctl_notebook = ttk.Notebook(right_col)
        ctl_notebook.pack(fill="both", expand=True)

        manual_tab = ttk.Frame(ctl_notebook, padding=15)
        ctl_notebook.add(manual_tab, text="🎮 Manual Control")

        action_row = ttk.Frame(manual_tab)
        action_row.pack(fill="x", pady=(0, 15))

        self.btn_stop = ttk.Button(
            action_row,
            text="🛑 EMERGENCY STOP",
            command=self.emergency_stop,
            bootstyle="danger",
            width=20
        )
        self.btn_stop.pack(side="left", fill="x", expand=True, ipady=5)

        self.btn_drop = ttk.Button(
            action_row,
            text="📦 DROP",
            command=lambda: self._run_cmd(self.drop_cmd),
            bootstyle="info",
            width=12,
        )
        self.btn_drop.pack(side="left", padx=(10, 0), ipady=5)

        self.btn_pick = ttk.Button(
            action_row,
            text="📥 PICK",
            command=lambda: self._run_cmd(self.pick_cmd),
            bootstyle="success",
            width=12,
        )
        self.btn_pick.pack(side="left", padx=(10, 0), ipady=5)

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

        # 3x3 layout
        self.btn_front.grid(row=0, column=1, padx=5, pady=5, ipadx=10, ipady=10)

        self.btn_left.grid(row=1, column=0, padx=5, pady=5, ipadx=10, ipady=10)
        ttk.Label(dpad, text="●").grid(row=1, column=1)
        self.btn_right.grid(row=1, column=2, padx=5, pady=5, ipadx=10, ipady=10)

        self.btn_back.grid(row=2, column=1, padx=5, pady=5, ipadx=10, ipady=10)

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

        self.btn_ccw.pack(side="left", padx=8, pady=5, ipadx=5, ipady=5)
        self.btn_cw.pack(side="left", padx=8, pady=5, ipadx=5, ipady=5)

        # ------------------------------
        # Go To Coordinate
        goto_frame = ttk.Labelframe(manual_tab, text="📌 Go To Coordinate", padding=15, bootstyle="info")
        goto_frame.pack(fill="x", pady=(15, 0))

        row1 = ttk.Frame(goto_frame)
        row1.pack(fill="x", pady=(0, 8))

        ttk.Label(row1, text="X (m):").pack(side="left")
        ttk.Entry(row1, textvariable=self.goto_x, width=10).pack(side="left", padx=(5, 12))

        ttk.Label(row1, text="Y (m):").pack(side="left")
        ttk.Entry(row1, textvariable=self.goto_y, width=10).pack(side="left", padx=(5, 12))

        ttk.Label(row1, text="Theta (rad, optional):").pack(side="left")
        ttk.Entry(row1, textvariable=self.goto_theta, width=12).pack(side="left", padx=(5, 0))

        row2 = ttk.Frame(goto_frame)
        row2.pack(fill="x", pady=(0, 8))

        self.btn_use_pose = ttk.Button(
            row2,
            text="📍 Use Current Pose",
            command=lambda: self._run_cmd(self.use_current_pose_cmd),
            bootstyle="secondary-outline",
            width=18
        )
        self.btn_use_pose.pack(side="left")

        self.btn_goto = ttk.Button(
            row2,
            text="🚀 GO",
            command=lambda: self._run_cmd(self.goto_coordinate_cmd),
            bootstyle="info",
            width=12
        )
        self.btn_goto.pack(side="right")

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

    def _set_controls_state(self, enabled: bool):
        state = "normal" if enabled else "disabled"
        for b in [
            self.reset_btn,
            self.btn_front, self.btn_back, self.btn_left, self.btn_right,
            self.btn_cw, self.btn_ccw,
            self.btn_stop, self.btn_drop, self.btn_pick, self.btn_goto, self.btn_use_pose, self.btn_clear_trail
        ]:
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
            self._log_line(f"ERROR[{level}] {err_type}: {desc}" if desc else f"ERROR[{level}] {err_type}")

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
                    self.after(0, lambda p=pose: self._update_pose_ui(p))
                else:
                    self.after(0, lambda: self._set_status("WAITING_POSE", "warning"))

                self.after(0, lambda s=state: self._log_state_errors(s))
            except Exception:
                self.after(0, lambda: self._set_status("WAITING_STATE", "warning"))
            time.sleep(0.25)

    def _update_pose_ui(self, pose):
        self.pose_x.set(f"{pose.x:.3f}")
        self.pose_y.set(f"{pose.y:.3f}")
        self.pose_theta.set(f"{math.degrees(pose.theta):.1f}")
        self.pose_map.set(f"{pose.map_id}")
        self.pose_desc.set(f"{pose.map_description}")
        self._push_trail_point(pose)
        self._redraw_trail()
        self._append_pose_log(pose)
        self._set_status("ONLINE", "success")

    def _push_trail_point(self, pose):
        p = (float(pose.x), float(pose.y), float(pose.theta))
        if self._trail_points and abs(self._trail_points[-1][0] - p[0]) < 1e-9 and abs(self._trail_points[-1][1] - p[1]) < 1e-9:
            self._trail_points[-1] = p
            return
        self._trail_points.append(p)
        if len(self._trail_points) > self._trail_max_points:
            self._trail_points = self._trail_points[-self._trail_max_points:]

    def clear_trail(self):
        self._trail_points.clear()
        self._redraw_trail()
        self._log_line("Trajectory cleared.")

    def _trail_bounds(self):
        if not self._trail_points:
            return (-1.0, 1.0, -1.0, 1.0)
        xs = [p[0] for p in self._trail_points]
        ys = [p[1] for p in self._trail_points]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        span = max(max_x - min_x, max_y - min_y, 1.0)
        pad = span * 0.15
        cx = (min_x + max_x) / 2.0
        cy = (min_y + max_y) / 2.0
        half = span / 2.0 + pad
        return (cx - half, cx + half, cy - half, cy + half)

    def _world_to_canvas(self, x, y, bounds, width, height):
        min_x, max_x, min_y, max_y = bounds
        world_w = max(max_x - min_x, 1e-6)
        world_h = max(max_y - min_y, 1e-6)
        sx = width / world_w
        sy = height / world_h
        scale = min(sx, sy) * 0.9
        cx = width * 0.5 + (x - (min_x + max_x) * 0.5) * scale
        cy = height * 0.5 - (y - (min_y + max_y) * 0.5) * scale
        return cx, cy, scale

    def _redraw_trail(self):
        if not hasattr(self, "trail_canvas"):
            return
        w = int(self.trail_canvas.winfo_width())
        h = int(self.trail_canvas.winfo_height())
        if w <= 2 or h <= 2:
            return

        self.trail_canvas.delete("all")
        self.trail_canvas.create_rectangle(0, 0, w, h, outline="#2a3f4f")

        bounds = self._trail_bounds()
        self.trail_canvas.create_line(0, h * 0.5, w, h * 0.5, fill="#1f2d3a")
        self.trail_canvas.create_line(w * 0.5, 0, w * 0.5, h, fill="#1f2d3a")

        if len(self._trail_points) >= 2:
            points = []
            for x, y, _theta in self._trail_points:
                cx, cy, _ = self._world_to_canvas(x, y, bounds, w, h)
                points.extend([cx, cy])
            self.trail_canvas.create_line(*points, fill="#35d0ff", width=2, smooth=True)

        if self._trail_points:
            x, y, theta = self._trail_points[-1]
            cx, cy, scale = self._world_to_canvas(x, y, bounds, w, h)
            r = 5
            self.trail_canvas.create_oval(cx - r, cy - r, cx + r, cy + r, fill="#ffcc00", outline="")
            arrow_len = max(12, min(28, int(0.35 * scale)))
            ex = cx + math.cos(theta) * arrow_len
            ey = cy - math.sin(theta) * arrow_len
            self.trail_canvas.create_line(cx, cy, ex, ey, fill="#ff8c42", width=3, arrow="last")

        self.trail_canvas.create_text(8, 8, anchor="nw", fill="#8fa4b6", text="Top view (m)")

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
        for b in [
            self.reset_btn,
            self.btn_front, self.btn_back, self.btn_left, self.btn_right,
            self.btn_cw, self.btn_ccw,
            self.btn_drop, self.btn_pick, self.btn_goto, self.btn_use_pose, self.btn_clear_trail
        ]:
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
                self.after(0, lambda: self._log_line("Stop sent."))
            except Exception as ex:
                self.after(0, lambda: self._log_line(f"Stop error: {ex}"))

        threading.Thread(target=worker, daemon=True).start()

    def drop_cmd(self):
        self._log_line("Send DROP action.")
        mc.send_drop_order()

    def pick_cmd(self):
        self._log_line("Send PICK action.")
        mc.send_pick_order()

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
        mc.manualMove(d, mc.MOVE_FORWARD)

    def move_back(self):
        d = self._get_move_val()
        self._log_line(f"Move back: {d} m")
        mc.manualMove(d, mc.MOVE_BACKWARD)

    def move_left(self):
        d = self._get_move_val()
        self._log_line(f"Move left: {d} m")
        mc.manualMove(d, mc.MOVE_LEFT)

    def move_right(self):
        d = self._get_move_val()
        self._log_line(f"Move right: {d} m")
        mc.manualMove(d, mc.MOVE_RIGHT)

    def rotate_cw(self):
        a = self._get_rot_val()
        self._log_line(f"Rotate CW: {a} deg")
        mc.manualRotate(a, mc.ROTATE_CLOCKWISE)

    def rotate_ccw(self):
        a = self._get_rot_val()
        self._log_line(f"Rotate CCW: {a} deg")
        mc.manualRotate(a, mc.ROTATE_COUNTERCLOCKWISE)

    def use_current_pose_cmd(self):
        if not self._connected:
            self._log_line("Not connected.")
            return
        pose = mc._get_latest_pose(timeout_s=2.0)
        self.goto_x.set(f"{pose.x:.3f}")
        self.goto_y.set(f"{pose.y:.3f}")
        self.goto_theta.set(f"{pose.theta:.6f}")
        self._log_line("Filled GoTo fields from current pose.")

    def goto_coordinate_cmd(self):
        try:
            x = float(self.goto_x.get().strip())
            y = float(self.goto_y.get().strip())
        except Exception:
            self._log_line("GoTo error: X/Y must be numeric.")
            return

        theta_txt = self.goto_theta.get().strip()
        theta_rad = None
        if theta_txt:
            try:
                theta_rad = float(theta_txt)
            except Exception:
                self._log_line("GoTo error: Theta must be numeric (rad) or empty.")
                return

        self._log_line(
            f"GoTo: x={x:.3f}, y={y:.3f}, theta(rad)={theta_rad if theta_rad is not None else 'auto'}"
        )
        mc.goto_coordinate(x, y, theta_rad=theta_rad)

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
