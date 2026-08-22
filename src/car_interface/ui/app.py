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

BACKGROUND = "#07111f"
SURFACE = "#0d1b2a"
CARD = "#122438"
CARD_HOVER = "#183149"
BORDER = "#29455f"
TEXT = "#f3f7fb"
MUTED = "#91a4b7"
PRIMARY = "#38bdf8"
PRIMARY_ACTIVE = "#7dd3fc"
SUCCESS = "#34d399"
WARNING = "#fbbf24"
DANGER = "#fb7185"
CANVAS_BACKGROUND = "#08131f"


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
        super().__init__(root, padding=18, style="App.TFrame")
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
        self.root.configure(background=BACKGROUND)

    def _configure_styles(self) -> None:
        style = ttk.Style(self.root)
        with suppress(tk.TclError):
            style.theme_use("clam")
        style.configure(".", font=("Segoe UI", 10), background=BACKGROUND, foreground=TEXT)
        style.configure("App.TFrame", background=BACKGROUND)
        style.configure("Header.TFrame", background=BACKGROUND)
        style.configure("Card.TFrame", background=CARD, relief=tk.FLAT)
        style.configure("Surface.TFrame", background=SURFACE, relief=tk.FLAT)
        style.configure(
            "Card.TLabelframe",
            background=CARD,
            bordercolor=BORDER,
            lightcolor=BORDER,
            darkcolor=BORDER,
            relief=tk.SOLID,
        )
        style.configure(
            "Card.TLabelframe.Label",
            background=CARD,
            foreground=TEXT,
            font=("Segoe UI", 10, "bold"),
        )
        style.configure(
            "Title.TLabel",
            background=BACKGROUND,
            foreground=TEXT,
            font=("Segoe UI", 21, "bold"),
        )
        style.configure(
            "Subtitle.TLabel",
            background=BACKGROUND,
            foreground=MUTED,
            font=("Segoe UI", 9),
        )
        style.configure(
            "PanelTitle.TLabel",
            background=CARD,
            foreground=TEXT,
            font=("Segoe UI", 11, "bold"),
        )
        style.configure("Body.TLabel", background=CARD, foreground=TEXT)
        style.configure("Muted.TLabel", background=CARD, foreground=MUTED)
        style.configure(
            "Metric.TLabel",
            background=CARD,
            foreground=TEXT,
            font=("Segoe UI", 18, "bold"),
        )
        style.configure(
            "MetricName.TLabel",
            background=CARD,
            foreground=MUTED,
            font=("Segoe UI", 8, "bold"),
        )
        style.configure(
            "Mode.TLabel",
            background="#12344a",
            foreground=PRIMARY,
            font=("Segoe UI", 9, "bold"),
            padding=(12, 7),
        )
        style.configure(
            "Status.TLabel",
            background=SURFACE,
            foreground=MUTED,
            font=("Segoe UI", 10, "bold"),
            padding=(10, 5),
        )
        style.configure(
            "Safe.TLabel",
            background="#103c38",
            foreground=SUCCESS,
            font=("Segoe UI", 10, "bold"),
            padding=(10, 5),
        )
        style.configure(
            "Warning.TLabel",
            background="#493715",
            foreground=WARNING,
            font=("Segoe UI", 10, "bold"),
            padding=(10, 5),
        )
        style.configure(
            "Danger.TLabel",
            background="#4b1f2b",
            foreground=DANGER,
            font=("Segoe UI", 10, "bold"),
            padding=(10, 5),
        )
        style.configure(
            "DeviceOff.TLabel",
            background=CARD,
            foreground=MUTED,
            font=("Segoe UI", 9, "bold"),
            padding=(7, 4),
        )
        style.configure(
            "DeviceOn.TLabel",
            background=CARD,
            foreground=SUCCESS,
            font=("Segoe UI", 9, "bold"),
            padding=(7, 4),
        )
        style.configure(
            "Primary.TButton",
            background=PRIMARY,
            foreground=BACKGROUND,
            borderwidth=0,
            focuscolor=PRIMARY,
            font=("Segoe UI", 9, "bold"),
            padding=(14, 6),
        )
        style.map(
            "Primary.TButton",
            background=[("active", PRIMARY_ACTIVE), ("disabled", "#274052")],
            foreground=[("disabled", "#718596")],
        )
        style.configure(
            "Secondary.TButton",
            background="#1c3348",
            foreground=TEXT,
            bordercolor=BORDER,
            focuscolor="#1c3348",
            font=("Segoe UI", 9, "bold"),
            padding=(14, 6),
        )
        style.map(
            "Secondary.TButton",
            background=[("active", CARD_HOVER), ("disabled", "#142638")],
            foreground=[("disabled", "#617687")],
        )
        style.configure(
            "Brake.TButton",
            background="#5b4218",
            foreground=WARNING,
            borderwidth=0,
            focuscolor="#5b4218",
            font=("Segoe UI", 9, "bold"),
            padding=(14, 6),
        )
        style.map("Brake.TButton", background=[("active", "#73541f")])
        style.configure(
            "Emergency.TButton",
            background="#be123c",
            foreground="#ffffff",
            borderwidth=0,
            focuscolor="#be123c",
            font=("Segoe UI", 13, "bold"),
            padding=(18, 8),
        )
        style.map(
            "Emergency.TButton",
            background=[("active", "#e11d48"), ("disabled", "#552436")],
            foreground=[("disabled", "#a87b89")],
        )
        style.configure("TNotebook", background=BACKGROUND, borderwidth=0, tabmargins=(0, 0, 0, 0))
        style.configure(
            "TNotebook.Tab",
            background=SURFACE,
            foreground=MUTED,
            borderwidth=0,
            font=("Segoe UI", 9, "bold"),
            padding=(18, 9),
        )
        style.map(
            "TNotebook.Tab",
            background=[("selected", CARD), ("active", CARD_HOVER)],
            foreground=[("selected", PRIMARY), ("active", TEXT)],
        )
        style.configure(
            "Horizontal.TScale",
            background=CARD,
            troughcolor="#20394f",
            bordercolor=CARD,
            lightcolor=PRIMARY,
            darkcolor=PRIMARY,
        )
        style.configure("TCheckbutton", background=CARD, foreground=TEXT)
        style.map("TCheckbutton", background=[("active", CARD)], foreground=[("active", TEXT)])
        style.configure("TRadiobutton", background=CARD, foreground=TEXT)
        style.map("TRadiobutton", background=[("active", CARD)], foreground=[("active", TEXT)])
        style.configure(
            "Vertical.TScrollbar",
            background="#29445c",
            troughcolor=CANVAS_BACKGROUND,
            bordercolor=CANVAS_BACKGROUND,
            arrowcolor=MUTED,
        )

    def _build_widgets(self) -> None:
        header = ttk.Frame(self, style="Header.TFrame")
        header.pack(fill=tk.X, pady=(0, 12))
        header.columnconfigure(0, weight=1)
        brand = ttk.Frame(header, style="Header.TFrame")
        brand.grid(row=0, column=0, sticky=tk.W)
        ttk.Label(
            brand,
            text="Autonomous Vehicle Control",
            style="Title.TLabel",
        ).pack(anchor=tk.W)
        ttk.Label(
            brand,
            text="CONTROL  ·  PERCEPTION  ·  DIAGNOSTICS",
            style="Subtitle.TLabel",
        ).pack(anchor=tk.W, pady=(2, 0))
        mode = "Simulation — no physical output" if self.simulation else "LIVE HARDWARE"
        self.mode_badge = ttk.Label(header, text=mode.upper(), style="Mode.TLabel")
        self.mode_badge.grid(row=0, column=1, sticky=tk.E, padx=(14, 0))

        overview = ttk.Frame(self, style="Card.TFrame", padding=(14, 10))
        overview.pack(fill=tk.X, pady=(0, 12))
        overview.columnconfigure(1, weight=1)
        state_group = ttk.Frame(overview, style="Card.TFrame")
        state_group.grid(row=0, column=0, sticky=tk.W)
        ttk.Label(state_group, text="SYSTEM STATE", style="MetricName.TLabel").pack(anchor=tk.W)
        self.phase_label = ttk.Label(
            state_group,
            text="State: DISCONNECTED",
            style="Status.TLabel",
        )
        self.phase_label.pack(anchor=tk.W, pady=(4, 0))

        self.fault_label = ttk.Label(overview, text="", style="Danger.TLabel")
        self.fault_label.grid(row=0, column=1, padx=18)

        devices = ttk.Frame(overview, style="Card.TFrame")
        devices.grid(row=0, column=2, sticky=tk.E)
        ttk.Label(devices, text="DEVICE LINKS", style="MetricName.TLabel").pack(anchor=tk.E)
        device_badges = ttk.Frame(devices, style="Card.TFrame")
        device_badges.pack(anchor=tk.E, pady=(3, 0))
        self.vehicle_status_label = ttk.Label(
            device_badges,
            text="○ Vehicle",
            style="DeviceOff.TLabel",
        )
        self.vehicle_status_label.pack(side=tk.LEFT)
        self.lidar_status_label = ttk.Label(
            device_badges,
            text="○ Lidar",
            style="DeviceOff.TLabel",
        )
        self.lidar_status_label.pack(side=tk.LEFT)
        self.controller_status_label = ttk.Label(
            device_badges,
            text="○ Controller",
            style="DeviceOff.TLabel",
        )
        self.controller_status_label.pack(side=tk.LEFT)
        self.connection_label = ttk.Label(
            devices,
            text="Vehicle ○  Lidar ○  Controller ○",
            style="Muted.TLabel",
        )

        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        self.control_tab = ttk.Frame(self.notebook, padding=12, style="App.TFrame")
        self.lidar_tab = ttk.Frame(self.notebook, padding=12, style="App.TFrame")
        self.log_tab = ttk.Frame(self.notebook, padding=12, style="App.TFrame")
        self.notebook.add(self.control_tab, text="Control")
        self.notebook.add(self.lidar_tab, text="Lidar & Safety")
        self.notebook.add(self.log_tab, text="Diagnostics")
        self._build_control_tab()
        self._build_lidar_tab()
        self._build_log_tab()

    def _build_control_tab(self) -> None:
        connection = ttk.LabelFrame(
            self.control_tab,
            text="Device lifecycle",
            padding=8,
            style="Card.TLabelframe",
        )
        connection.pack(fill=tk.X, pady=(0, 10))
        self.connect_button = ttk.Button(
            connection,
            text="Connect systems",
            style="Primary.TButton",
            command=self.service.connect,
        )
        self.connect_button.pack(side=tk.LEFT, padx=4)
        self.disconnect_button = ttk.Button(
            connection,
            text="Safe disconnect",
            style="Secondary.TButton",
            command=self.service.disconnect,
        )
        self.disconnect_button.pack(side=tk.LEFT, padx=4)
        self.arm_button = ttk.Button(
            connection,
            text="Arm drive",
            style="Primary.TButton",
            command=self.service.arm,
        )
        self.arm_button.pack(side=tk.LEFT, padx=4)
        self.disarm_button = ttk.Button(
            connection,
            text="Disarm",
            style="Secondary.TButton",
            command=self.service.disarm,
        )
        self.disarm_button.pack(side=tk.LEFT, padx=4)
        ttk.Label(
            connection,
            text="Connect all simulated or physical subsystems before arming.",
            style="Muted.TLabel",
        ).pack(side=tk.RIGHT, padx=4)

        emergency = ttk.LabelFrame(
            self.control_tab,
            text="Emergency controls",
            padding=8,
            style="Card.TLabelframe",
        )
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
            style="Secondary.TButton",
            command=self._confirm_estop_reset,
        )
        self.reset_estop_button.pack(side=tk.RIGHT)

        movement = ttk.LabelFrame(
            self.control_tab,
            text="Motion command",
            padding=10,
            style="Card.TLabelframe",
        )
        movement.pack(fill=tk.BOTH, expand=True)
        movement.columnconfigure(0, weight=1)
        movement.columnconfigure(1, weight=1)

        self.speed_value = tk.IntVar(value=0)
        speed_heading = ttk.Frame(movement, style="Card.TFrame")
        speed_heading.grid(row=0, column=0, sticky=tk.EW, padx=(0, 14))
        ttk.Label(speed_heading, text="COMMAND SPEED", style="MetricName.TLabel").pack(side=tk.LEFT)
        self.speed_label = ttk.Label(speed_heading, text="0%", style="Metric.TLabel")
        self.speed_label.pack(side=tk.RIGHT)
        self.speed_scale = ttk.Scale(
            movement,
            from_=0,
            to=self.max_speed_percent,
            variable=self.speed_value,
            command=self._on_speed,
        )
        self.speed_scale.grid(row=1, column=0, sticky=tk.EW, padx=(0, 14), pady=(8, 3))
        ttk.Label(
            movement,
            text=f"0 — {self.max_speed_percent}% configured limit",
            style="Muted.TLabel",
        ).grid(row=2, column=0, sticky=tk.W)

        self.steering_value = tk.IntVar(value=0)
        steering_heading = ttk.Frame(movement, style="Card.TFrame")
        steering_heading.grid(row=0, column=1, sticky=tk.EW)
        ttk.Label(steering_heading, text="STEERING", style="MetricName.TLabel").pack(side=tk.LEFT)
        self.steering_label = ttk.Label(steering_heading, text="0%", style="Metric.TLabel")
        self.steering_label.pack(side=tk.RIGHT)
        self.steering_scale = ttk.Scale(
            movement,
            from_=-100,
            to=100,
            variable=self.steering_value,
            command=self._on_steering,
        )
        self.steering_scale.grid(row=1, column=1, sticky=tk.EW, pady=(8, 3))
        ttk.Label(
            movement,
            text="-100% left  ·  0% center  ·  +100% right",
            style="Muted.TLabel",
        ).grid(row=2, column=1, sticky=tk.W)

        direction_frame = ttk.Frame(movement, style="Card.TFrame")
        direction_frame.grid(row=3, column=0, columnspan=2, sticky=tk.EW, pady=(8, 0))
        self.direction_value = tk.StringVar(value="forward")
        direction_controls = ttk.Frame(direction_frame, style="Card.TFrame")
        direction_controls.pack(side=tk.LEFT)
        ttk.Label(direction_controls, text="DIRECTION", style="MetricName.TLabel").pack(
            side=tk.LEFT, padx=(0, 8)
        )
        ttk.Radiobutton(
            direction_controls,
            text="Forward",
            value="forward",
            variable=self.direction_value,
            command=self._on_direction,
        ).pack(side=tk.LEFT, padx=4)
        ttk.Radiobutton(
            direction_controls,
            text="Reverse",
            value="reverse",
            variable=self.direction_value,
            command=self._on_direction,
        ).pack(side=tk.LEFT, padx=4)
        ttk.Button(
            direction_controls,
            text="Apply brake",
            style="Brake.TButton",
            command=lambda: self.service.set_brake(True),
        ).pack(side=tk.LEFT, padx=(10, 4))

        telemetry = ttk.Frame(direction_frame, style="Surface.TFrame", padding=(4, 2))
        telemetry.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=(12, 0))
        for column in range(4):
            telemetry.columnconfigure(column, weight=1)
        self.live_speed_label = self._build_telemetry_metric(
            telemetry,
            column=0,
            name="LIVE SPEED",
            value="0%",
        )
        self.live_steering_label = self._build_telemetry_metric(
            telemetry,
            column=1,
            name="LIVE STEERING",
            value="0%",
        )
        self.direction_status_label = self._build_telemetry_metric(
            telemetry,
            column=2,
            name="DIRECTION",
            value="FORWARD",
        )
        self.brake_status_label = self._build_telemetry_metric(
            telemetry,
            column=3,
            name="BRAKE",
            value="ENGAGED",
        )

    @staticmethod
    def _build_telemetry_metric(
        parent: ttk.Frame,
        *,
        column: int,
        name: str,
        value: str,
    ) -> ttk.Label:
        metric = ttk.Frame(parent, style="Card.TFrame", padding=(6, 1))
        metric.grid(row=0, column=column, sticky=tk.EW, padx=3)
        ttk.Label(metric, text=name, style="MetricName.TLabel").pack(anchor=tk.W)
        value_label = ttk.Label(metric, text=value, style="PanelTitle.TLabel")
        value_label.pack(anchor=tk.W, pady=(1, 0))
        return value_label

    def _build_lidar_tab(self) -> None:
        settings = ttk.LabelFrame(
            self.lidar_tab,
            text="Perception status",
            padding=10,
            style="Card.TLabelframe",
        )
        settings.pack(fill=tk.X, pady=(0, 10))
        self.auto_stop_value = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            settings,
            text="Enable automatic obstacle E-stop",
            variable=self.auto_stop_value,
            command=self._on_auto_stop_changed,
        ).pack(side=tk.LEFT)
        self.obstacle_label = ttk.Label(
            settings,
            text="Closest path obstacle: —",
            style="PanelTitle.TLabel",
        )
        self.obstacle_label.pack(side=tk.RIGHT)

        if self.simulation:
            simulator = ttk.LabelFrame(
                self.lidar_tab,
                text="Simulation scenario",
                padding=10,
                style="Card.TLabelframe",
            )
            simulator.pack(fill=tk.X, pady=(0, 10))
            self.simulated_distance = tk.DoubleVar(value=180.0)
            ttk.Label(
                simulator,
                text="Obstacle distance",
                style="Body.TLabel",
            ).pack(side=tk.LEFT)
            ttk.Scale(
                simulator,
                from_=20,
                to=400,
                variable=self.simulated_distance,
                command=lambda value: self.service.set_simulated_obstacle(float(value)),
            ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=10)
            ttk.Label(simulator, text="20 — 400 cm", style="Muted.TLabel").pack(side=tk.RIGHT)

        self.lidar_canvas = tk.Canvas(
            self.lidar_tab,
            background=CANVAS_BACKGROUND,
            highlightbackground=BORDER,
            highlightcolor=PRIMARY,
            highlightthickness=1,
        )
        self.lidar_canvas.pack(fill=tk.BOTH, expand=True)
        self.lidar_canvas.bind("<Configure>", lambda _event: self._draw_lidar())

    def _build_log_tab(self) -> None:
        toolbar = ttk.Frame(self.log_tab, style="Card.TFrame", padding=(12, 9))
        toolbar.pack(fill=tk.X, pady=(0, 8))
        heading = ttk.Frame(toolbar, style="Card.TFrame")
        heading.pack(side=tk.LEFT)
        ttk.Label(heading, text="Live event stream", style="PanelTitle.TLabel").pack(anchor=tk.W)
        ttk.Label(
            heading,
            text="Connection, command, and fault events from the control service",
            style="Muted.TLabel",
        ).pack(anchor=tk.W)
        ttk.Button(
            toolbar,
            text="Clear events",
            style="Secondary.TButton",
            command=self._clear_log,
        ).pack(side=tk.RIGHT)
        self.log_text = tk.Text(
            self.log_tab,
            wrap=tk.WORD,
            state=tk.DISABLED,
            background=CANVAS_BACKGROUND,
            foreground=TEXT,
            insertbackground=TEXT,
            selectbackground="#1f5f7e",
            selectforeground="#ffffff",
            borderwidth=0,
            highlightbackground=BORDER,
            highlightcolor=PRIMARY,
            highlightthickness=1,
            padx=12,
            pady=10,
            spacing1=2,
            spacing3=2,
            font=("Consolas", 9),
        )
        self.log_text.tag_configure(EventType.LOG.value, foreground=MUTED)
        self.log_text.tag_configure(EventType.COMMAND.value, foreground=PRIMARY)
        self.log_text.tag_configure(EventType.CONNECTION.value, foreground=SUCCESS)
        self.log_text.tag_configure(EventType.FAULT.value, foreground=DANGER)
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
            message = str(event.payload)
            if event.type is not EventType.COMMAND or not message.endswith(" HBT"):
                self._append_log(message, event.type)

    def _apply_snapshot(self, snapshot: ServiceSnapshot) -> None:
        phase = snapshot.phase.upper()
        self.phase_label.config(text=f"State: {phase}")
        if snapshot.estop_active or snapshot.fault:
            phase_style = "Danger.TLabel"
        elif snapshot.phase == "braking":
            phase_style = "Warning.TLabel"
        elif snapshot.phase in {"safe_connected", "armed", "driving"}:
            phase_style = "Safe.TLabel"
        else:
            phase_style = "Status.TLabel"
        self.phase_label.config(style=phase_style)
        self.connection_label.config(
            text=(
                f"Vehicle {'●' if snapshot.vehicle_connected else '○'}  "
                f"Lidar {'●' if snapshot.lidar_connected else '○'}  "
                f"Controller {'●' if snapshot.controller_connected else '○'}"
            )
        )
        self._update_device_badge(
            self.vehicle_status_label,
            name="Vehicle",
            connected=snapshot.vehicle_connected,
        )
        self._update_device_badge(
            self.lidar_status_label,
            name="Lidar",
            connected=snapshot.lidar_connected,
        )
        self._update_device_badge(
            self.controller_status_label,
            name="Controller",
            connected=snapshot.controller_connected,
        )
        self.fault_label.config(text=snapshot.fault or "")
        if snapshot.fault:
            self.fault_label.grid()
        else:
            self.fault_label.grid_remove()
        closest = snapshot.closest_obstacle_cm
        self.obstacle_label.config(
            text="Closest path obstacle: —"
            if closest is None
            else f"Closest path obstacle: {closest:.1f} cm"
        )
        self.auto_stop_value.set(snapshot.auto_stop_enabled)
        self.live_speed_label.config(text=f"{abs(snapshot.speed_percent)}%")
        steering = snapshot.steering_percent
        steering_text = "0%" if steering == 0 else f"{steering:+d}%"
        self.live_steering_label.config(text=steering_text)
        self.direction_status_label.config(text=snapshot.direction.upper())
        self.brake_status_label.config(
            text="ENGAGED" if snapshot.brake_active else "RELEASED",
            style="Warning.TLabel" if snapshot.brake_active else "Safe.TLabel",
        )
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

    @staticmethod
    def _update_device_badge(label: ttk.Label, *, name: str, connected: bool) -> None:
        marker = "●" if connected else "○"
        style = "DeviceOn.TLabel" if connected else "DeviceOff.TLabel"
        label.config(text=f"{marker} {name}", style=style)

    def _draw_lidar(self) -> None:
        canvas = self.lidar_canvas
        width = max(canvas.winfo_width(), 10)
        height = max(canvas.winfo_height(), 10)
        canvas.delete("all")
        center_x, center_y = width / 2.0, height * 0.92
        radius = min(width * 0.47, height * 0.82)
        max_distance = 400.0
        canvas.create_text(
            16,
            14,
            text="LIVE 180° SCAN",
            fill=TEXT,
            anchor=tk.NW,
            font=("Segoe UI", 10, "bold"),
            tags=("scan-title",),
        )
        canvas.create_text(
            16,
            34,
            text="Vehicle-relative range · 400 cm",
            fill=MUTED,
            anchor=tk.NW,
            font=("Segoe UI", 8),
            tags=("scan-subtitle",),
        )

        corridor_half_width = max(radius * 0.10, 18.0)
        canvas.create_polygon(
            center_x - corridor_half_width,
            center_y,
            center_x - corridor_half_width,
            center_y - radius,
            center_x + corridor_half_width,
            center_y - radius,
            center_x + corridor_half_width,
            center_y,
            fill="#0b2738",
            outline="#23617d",
            width=1,
            tags=("projected-path",),
        )
        for heading in (-60, -30, 30, 60):
            heading_radians = math.radians(heading - 90.0)
            canvas.create_line(
                center_x,
                center_y,
                center_x + math.cos(heading_radians) * radius,
                center_y + math.sin(heading_radians) * radius,
                fill="#183145",
                dash=(2, 6),
                tags=("bearing-grid",),
            )
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
                fill=MUTED,
                anchor=tk.SW,
                font=("Segoe UI", 8),
                tags=("range-label",),
            )
        canvas.create_line(
            center_x,
            center_y,
            center_x,
            center_y - radius,
            fill=PRIMARY,
            width=1,
            tags=("centerline",),
        )
        vehicle_width = 24.0
        vehicle_height = 34.0
        canvas.create_rectangle(
            center_x - vehicle_width / 2,
            center_y - vehicle_height,
            center_x + vehicle_width / 2,
            center_y,
            fill="#123e56",
            outline=PRIMARY,
            width=2,
            tags=("vehicle",),
        )
        canvas.create_polygon(
            center_x,
            center_y - vehicle_height - 8,
            center_x - 6,
            center_y - vehicle_height + 2,
            center_x + 6,
            center_y - vehicle_height + 2,
            fill=PRIMARY,
            outline="",
            tags=("vehicle-heading",),
        )
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
            canvas.create_oval(
                x - 2,
                y - 2,
                x + 2,
                y + 2,
                fill=color,
                outline="",
                tags=("scan-point",),
            )

        legend_x = width - 122
        for offset, (label, color) in enumerate(
            (("Near", DANGER), ("Caution", WARNING), ("Clear", SUCCESS))
        ):
            legend_y = 18 + offset * 20
            canvas.create_oval(
                legend_x,
                legend_y,
                legend_x + 7,
                legend_y + 7,
                fill=color,
                outline="",
                tags=("legend",),
            )
            canvas.create_text(
                legend_x + 14,
                legend_y + 4,
                text=label,
                fill=MUTED,
                anchor=tk.W,
                font=("Segoe UI", 8),
                tags=("legend",),
            )

    def _append_log(self, message: str, event_type: EventType) -> None:
        timestamp = datetime.now().astimezone().strftime("%H:%M:%S%z")
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(
            tk.END,
            f"[{timestamp}] {event_type.value.upper()}: {message}\n",
            event_type.value,
        )
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
