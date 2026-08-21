"""Responsive Tkinter UI for the production control service."""

from __future__ import annotations

import math
import threading
import tkinter as tk
from contextlib import suppress
from datetime import datetime
from tkinter import messagebox, ttk
from typing import Any, Protocol, cast

from car_interface.adapters.base import ScanPoint
from car_interface.services.events import EventBroker, EventType, ServiceEvent
from car_interface.services.lidar_analysis import LidarAssessment
from car_interface.services.models import ServiceSnapshot


class ControlServiceProtocol(Protocol):
    events: EventBroker

    def snapshot(self) -> ServiceSnapshot: ...

    def connect(self) -> None: ...

    def disconnect(self) -> None: ...

    def arm(self) -> None: ...

    def disarm(self) -> None: ...

    def set_speed(self, speed_percent: int) -> None: ...

    def set_steering(self, steering_percent: int) -> None: ...

    def set_direction(self, direction: str) -> None: ...

    def set_brake(self, active: bool) -> None: ...

    def emergency_stop(self, reason: str = "operator") -> None: ...

    def reset_emergency_stop(self) -> None: ...

    def set_auto_stop(self, enabled: bool) -> None: ...

    def set_simulated_obstacle(self, distance_cm: float) -> None: ...

    def shutdown(self) -> None: ...


class CarInterfaceWindow(ttk.Frame):
    """View-only GUI. Device and safety work stays inside the service."""

    REFRESH_MS = 50
    MAX_LOG_LINES = 2_000

    def __init__(
        self,
        root: tk.Tk,
        service: ControlServiceProtocol,
        *,
        simulation: bool,
        max_speed_percent: int,
    ) -> None:
        super().__init__(root, padding=10)
        self.root = root
        self.service = service
        self.simulation = simulation
        self.max_speed_percent = max_speed_percent
        self._closing = False
        self._shutdown_finished = False
        self._last_scan: tuple[ScanPoint, ...] = ()
        self._configure_window()
        self._configure_styles()
        self._build_widgets()
        self.pack(fill=tk.BOTH, expand=True)
        self.root.protocol("WM_DELETE_WINDOW", self._request_close)
        self.root.after(self.REFRESH_MS, self._refresh)

    def _configure_window(self) -> None:
        mode_label = "SIMULATION" if self.simulation else "HARDWARE"
        self.root.title(f"Car Interface — {mode_label}")
        self.root.geometry("1120x760")
        self.root.minsize(900, 640)

    def _configure_styles(self) -> None:
        style = ttk.Style(self.root)
        with suppress(tk.TclError):
            style.theme_use("clam")
        style.configure("Title.TLabel", font=("Segoe UI", 17, "bold"))
        style.configure("Status.TLabel", font=("Segoe UI", 10, "bold"))
        style.configure("Emergency.TButton", font=("Segoe UI", 14, "bold"))
        style.configure("Safe.TLabel", foreground="#087f23")
        style.configure("Warning.TLabel", foreground="#b35c00")
        style.configure("Danger.TLabel", foreground="#b00020")

    def _build_widgets(self) -> None:
        header = ttk.Frame(self)
        header.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(header, text="ESP32 Car Control", style="Title.TLabel").pack(side=tk.LEFT)
        mode = "Simulation — no physical output" if self.simulation else "LIVE HARDWARE"
        self.mode_badge = ttk.Label(header, text=mode, style="Status.TLabel")
        self.mode_badge.pack(side=tk.RIGHT)

        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        self.control_tab = ttk.Frame(self.notebook, padding=12)
        self.lidar_tab = ttk.Frame(self.notebook, padding=12)
        self.log_tab = ttk.Frame(self.notebook, padding=12)
        self.notebook.add(self.control_tab, text="Control")
        self.notebook.add(self.lidar_tab, text="Lidar & Safety")
        self.notebook.add(self.log_tab, text="Diagnostics")
        self._build_control_tab()
        self._build_lidar_tab()
        self._build_log_tab()

        status_frame = ttk.Frame(self)
        status_frame.pack(fill=tk.X, pady=(8, 0))
        self.phase_label = ttk.Label(status_frame, text="DISCONNECTED", style="Status.TLabel")
        self.phase_label.pack(side=tk.LEFT)
        self.fault_label = ttk.Label(status_frame, text="", style="Danger.TLabel")
        self.fault_label.pack(side=tk.RIGHT)

    def _build_control_tab(self) -> None:
        connection = ttk.LabelFrame(self.control_tab, text="Device lifecycle", padding=10)
        connection.pack(fill=tk.X, pady=(0, 10))
        self.connect_button = ttk.Button(connection, text="Connect", command=self.service.connect)
        self.connect_button.pack(side=tk.LEFT, padx=4)
        self.disconnect_button = ttk.Button(
            connection, text="Safe disconnect", command=self.service.disconnect
        )
        self.disconnect_button.pack(side=tk.LEFT, padx=4)
        self.arm_button = ttk.Button(connection, text="Arm", command=self.service.arm)
        self.arm_button.pack(side=tk.LEFT, padx=4)
        self.disarm_button = ttk.Button(connection, text="Disarm", command=self.service.disarm)
        self.disarm_button.pack(side=tk.LEFT, padx=4)
        self.connection_label = ttk.Label(connection, text="Vehicle ○  Lidar ○  Controller ○")
        self.connection_label.pack(side=tk.RIGHT, padx=4)

        emergency = ttk.LabelFrame(self.control_tab, text="Emergency controls", padding=10)
        emergency.pack(fill=tk.X, pady=(0, 10))
        self.estop_button = ttk.Button(
            emergency,
            text="EMERGENCY STOP",
            style="Emergency.TButton",
            command=lambda: self.service.emergency_stop("operator button"),
        )
        self.estop_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 6))
        self.reset_estop_button = ttk.Button(
            emergency,
            text="Reset E-stop / fault",
            command=self._confirm_estop_reset,
        )
        self.reset_estop_button.pack(side=tk.RIGHT)

        movement = ttk.LabelFrame(self.control_tab, text="Motion", padding=10)
        movement.pack(fill=tk.BOTH, expand=True)
        movement.columnconfigure(0, weight=1)
        movement.columnconfigure(1, weight=1)

        self.speed_value = tk.IntVar(value=0)
        ttk.Label(movement, text="Command speed (%)").grid(row=0, column=0, sticky=tk.W)
        self.speed_scale = ttk.Scale(
            movement,
            from_=0,
            to=self.max_speed_percent,
            variable=self.speed_value,
            command=self._on_speed,
        )
        self.speed_scale.grid(row=1, column=0, sticky=tk.EW, padx=(0, 12), pady=6)
        self.speed_label = ttk.Label(movement, text="0%")
        self.speed_label.grid(row=2, column=0, sticky=tk.W)

        self.steering_value = tk.IntVar(value=0)
        ttk.Label(movement, text="Steering (%)").grid(row=0, column=1, sticky=tk.W)
        self.steering_scale = ttk.Scale(
            movement,
            from_=-100,
            to=100,
            variable=self.steering_value,
            command=self._on_steering,
        )
        self.steering_scale.grid(row=1, column=1, sticky=tk.EW, pady=6)
        self.steering_label = ttk.Label(movement, text="0%")
        self.steering_label.grid(row=2, column=1, sticky=tk.W)

        direction_frame = ttk.Frame(movement)
        direction_frame.grid(row=3, column=0, columnspan=2, sticky=tk.EW, pady=12)
        self.direction_value = tk.StringVar(value="forward")
        ttk.Radiobutton(
            direction_frame,
            text="Forward",
            value="forward",
            variable=self.direction_value,
            command=self._on_direction,
        ).pack(side=tk.LEFT, padx=4)
        ttk.Radiobutton(
            direction_frame,
            text="Reverse",
            value="reverse",
            variable=self.direction_value,
            command=self._on_direction,
        ).pack(side=tk.LEFT, padx=4)
        ttk.Button(
            direction_frame,
            text="Apply brake",
            command=lambda: self.service.set_brake(True),
        ).pack(side=tk.RIGHT, padx=4)

    def _build_lidar_tab(self) -> None:
        settings = ttk.Frame(self.lidar_tab)
        settings.pack(fill=tk.X, pady=(0, 8))
        self.auto_stop_value = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            settings,
            text="Enable automatic obstacle E-stop",
            variable=self.auto_stop_value,
            command=self._on_auto_stop_changed,
        ).pack(side=tk.LEFT)
        self.obstacle_label = ttk.Label(settings, text="Closest path obstacle: —")
        self.obstacle_label.pack(side=tk.RIGHT)

        if self.simulation:
            simulator = ttk.LabelFrame(self.lidar_tab, text="Simulation fault injection", padding=8)
            simulator.pack(fill=tk.X, pady=(0, 8))
            self.simulated_distance = tk.DoubleVar(value=180.0)
            ttk.Label(simulator, text="Obstacle distance (cm)").pack(side=tk.LEFT)
            ttk.Scale(
                simulator,
                from_=20,
                to=400,
                variable=self.simulated_distance,
                command=lambda value: self.service.set_simulated_obstacle(float(value)),
            ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=10)

        self.lidar_canvas = tk.Canvas(
            self.lidar_tab,
            background="#111418",
            highlightthickness=0,
        )
        self.lidar_canvas.pack(fill=tk.BOTH, expand=True)
        self.lidar_canvas.bind("<Configure>", lambda _event: self._draw_lidar())

    def _build_log_tab(self) -> None:
        toolbar = ttk.Frame(self.log_tab)
        toolbar.pack(fill=tk.X, pady=(0, 6))
        ttk.Button(toolbar, text="Clear", command=self._clear_log).pack(side=tk.RIGHT)
        self.log_text = tk.Text(
            self.log_tab,
            wrap=tk.WORD,
            state=tk.DISABLED,
            background="#111418",
            foreground="#e8eaed",
            insertbackground="#e8eaed",
            font=("Consolas", 9),
        )
        log_scrollbar = ttk.Scrollbar(
            self.log_tab,
            orient=tk.VERTICAL,
            command=self.log_text.yview,
        )
        self.log_text.configure(yscrollcommand=log_scrollbar.set)
        log_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    def _on_speed(self, value: str) -> None:
        speed = round(float(value))
        self.speed_label.config(text=f"{speed}%")
        self.service.set_speed(speed)

    def _on_steering(self, value: str) -> None:
        steering = round(float(value))
        self.steering_label.config(text=f"{steering}%")
        self.service.set_steering(steering)

    def _on_direction(self) -> None:
        self.service.set_direction(self.direction_value.get())

    def _on_auto_stop_changed(self) -> None:
        enabled = self.auto_stop_value.get()
        if not enabled and not messagebox.askyesno(
            "Disable obstacle stop",
            "Disable automatic Lidar obstacle stopping? The Lidar remains required, but "
            "nearby obstacles will no longer trigger the software E-stop.",
            icon=messagebox.WARNING,
        ):
            self.auto_stop_value.set(True)
            return
        self.service.set_auto_stop(enabled)

    def _confirm_estop_reset(self) -> None:
        confirmed = messagebox.askyesno(
            "Reset safety latch",
            "Confirm the vehicle is stationary, the fault cause is corrected, the path "
            "is clear, and the physical emergency stop is ready before resetting.",
            icon=messagebox.WARNING,
        )
        if confirmed:
            self.service.reset_emergency_stop()

    def _refresh(self) -> None:
        if self._closing:
            return
        for event in self.service.events.drain():
            self._handle_event(event)
        self._apply_snapshot(self.service.snapshot())
        self.root.after(self.REFRESH_MS, self._refresh)

    def _handle_event(self, event: ServiceEvent) -> None:
        if event.type == EventType.SCAN:
            assessment = event.payload
            if isinstance(assessment, LidarAssessment):
                self._last_scan = assessment.points
                self._draw_lidar()
        elif event.type in {
            EventType.LOG,
            EventType.COMMAND,
            EventType.FAULT,
            EventType.CONNECTION,
        }:
            self._append_log(str(event.payload), event.type)

    def _apply_snapshot(self, snapshot: ServiceSnapshot) -> None:
        phase = snapshot.phase.upper()
        self.phase_label.config(text=f"State: {phase}")
        style = "Danger.TLabel" if snapshot.estop_active or snapshot.fault else "Safe.TLabel"
        self.phase_label.config(style=style)
        self.connection_label.config(
            text=(
                f"Vehicle {'●' if snapshot.vehicle_connected else '○'}  "
                f"Lidar {'●' if snapshot.lidar_connected else '○'}  "
                f"Controller {'●' if snapshot.controller_connected else '○'}"
            )
        )
        self.fault_label.config(text=snapshot.fault or "")
        closest = snapshot.closest_obstacle_cm
        self.obstacle_label.config(
            text="Closest path obstacle: —"
            if closest is None
            else f"Closest path obstacle: {closest:.1f} cm"
        )
        self.auto_stop_value.set(snapshot.auto_stop_enabled)
        if snapshot.estop_active or snapshot.brake_active:
            self.speed_value.set(0)
            self.speed_label.config(text="0%")
            self.steering_value.set(snapshot.steering_percent)
            self.steering_label.config(text=f"{snapshot.steering_percent}%")
        self.direction_value.set(snapshot.direction)

        self.connect_button.config(state=tk.DISABLED if snapshot.vehicle_connected else tk.NORMAL)
        self.disconnect_button.config(
            state=tk.NORMAL if snapshot.vehicle_connected else tk.DISABLED
        )
        armable = (
            snapshot.phase in {"safe_connected", "braking"}
            and snapshot.vehicle_connected
            and snapshot.lidar_connected
            and snapshot.controller_connected
        )
        self.arm_button.config(state=tk.NORMAL if armable else tk.DISABLED)
        self.disarm_button.config(
            state=tk.NORMAL if snapshot.phase in {"armed", "driving", "braking"} else tk.DISABLED
        )
        self.reset_estop_button.config(
            state=(tk.NORMAL if snapshot.estop_active or snapshot.phase == "fault" else tk.DISABLED)
        )
        motion_enabled = snapshot.phase in {"armed", "driving"} and not snapshot.estop_active
        scale_state = tk.NORMAL if motion_enabled else tk.DISABLED
        self.speed_scale.config(state=scale_state)
        self.steering_scale.config(state=scale_state)

    def _draw_lidar(self) -> None:
        canvas = self.lidar_canvas
        width = max(canvas.winfo_width(), 10)
        height = max(canvas.winfo_height(), 10)
        canvas.delete("all")
        center_x, center_y = width / 2.0, height * 0.92
        radius = min(width * 0.47, height * 0.82)
        max_distance = 400.0
        for distance in (50, 100, 200, 400):
            scaled = radius * distance / max_distance
            canvas.create_arc(
                center_x - scaled,
                center_y - scaled,
                center_x + scaled,
                center_y + scaled,
                start=0,
                extent=180,
                outline="#35404a",
                dash=(2, 4),
            )
            canvas.create_text(
                center_x + 4,
                center_y - scaled,
                text=f"{distance}cm",
                fill="#7f8c96",
                anchor=tk.SW,
            )
        canvas.create_line(center_x, center_y, center_x, center_y - radius, fill="#62727d")
        for point in self._last_scan:
            if point.distance_cm > max_distance:
                continue
            angle = math.radians(point.angle_degrees - 90.0)
            point_radius = radius * point.distance_cm / max_distance
            x = center_x + math.cos(angle) * point_radius
            y = center_y + math.sin(angle) * point_radius
            color = (
                "#ff5252"
                if point.distance_cm < 50
                else "#ffd740"
                if point.distance_cm < 150
                else "#69f0ae"
            )
            canvas.create_oval(x - 2, y - 2, x + 2, y + 2, fill=color, outline="")

    def _append_log(self, message: str, event_type: EventType) -> None:
        timestamp = datetime.now().astimezone().strftime("%H:%M:%S%z")
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{timestamp}] {event_type.value.upper()}: {message}\n")
        line_count = int(self.log_text.index("end-1c").partition(".")[0])
        if line_count > self.MAX_LOG_LINES:
            self.log_text.delete("1.0", f"{line_count - self.MAX_LOG_LINES + 1}.0")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)

    def _clear_log(self) -> None:
        self.log_text.config(state=tk.NORMAL)
        self.log_text.delete("1.0", tk.END)
        self.log_text.config(state=tk.DISABLED)

    def _request_close(self) -> None:
        if self._closing:
            return
        if not messagebox.askokcancel(
            "Exit Car Interface",
            "The application will command a safe stop before disconnecting. Exit now?",
        ):
            return
        self._closing = True
        for child in self.winfo_children():
            with suppress(tk.TclError, AttributeError):
                cast(Any, child).configure(state=tk.DISABLED)
        threading.Thread(target=self._shutdown_worker, name="ui-shutdown", daemon=True).start()
        self.root.after(50, self._wait_for_shutdown)

    def _shutdown_worker(self) -> None:
        try:
            self.service.shutdown()
        finally:
            self._shutdown_finished = True

    def _wait_for_shutdown(self) -> None:
        if self._shutdown_finished:
            self.root.destroy()
        else:
            self.root.after(50, self._wait_for_shutdown)
