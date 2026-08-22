"""Deterministic simulator walkthrough used by reviewers and media capture."""

from __future__ import annotations

import time
import tkinter as tk
from collections.abc import Callable
from typing import TYPE_CHECKING, Any, Protocol, cast

from car_interface.services.models import ServiceSnapshot

if TYPE_CHECKING:
    from .app import CarInterfaceWindow


class ShowcaseService(Protocol):
    """Control operations exercised by the simulator walkthrough."""

    def snapshot(self) -> ServiceSnapshot: ...

    def connect(self) -> None: ...

    def arm(self) -> None: ...

    def set_speed(self, speed_percent: int) -> None: ...

    def set_steering(self, steering_percent: int) -> None: ...

    def set_simulated_obstacle(self, distance_cm: float) -> None: ...

    def reset_emergency_stop(self) -> None: ...


StageCallback = Callable[[str], None]


class ShowcaseDirector:
    """Traverse the real simulator through a repeatable operator story.

    The director only sequences public control-service operations and existing
    presentation widgets. It does not bypass the state machine or inject fake
    snapshots, so every state shown in the walkthrough is produced by the
    application itself.
    """

    CONNECTION_TIMEOUT_MS = 6_000
    TRANSITION_TIMEOUT_MS = 3_000
    POLL_MS = 100

    def __init__(
        self,
        root: tk.Misc,
        window: CarInterfaceWindow,
        service: ShowcaseService,
        *,
        on_stage: StageCallback | None = None,
        on_complete: Callable[[], None] | None = None,
    ) -> None:
        if not window.simulation:
            raise ValueError("the showcase is available only in simulation mode")
        self.root = root
        self.window = window
        self.service = service
        self.on_stage = on_stage
        self.on_complete = on_complete
        self._started = False
        self._stopped = False
        self._finished = False

    def start(self) -> None:
        """Start the walkthrough from a disconnected simulator."""

        if self._started or self._stopped or self._finished:
            return
        self._started = True
        self._select_tab(self.window.control_tab)
        self._announce("connecting")
        self.service.connect()
        self._wait_for(
            lambda: self._all_devices_connected(self.service.snapshot()),
            self._connected,
            timeout_ms=self.CONNECTION_TIMEOUT_MS,
        )

    def stop(self) -> None:
        """Prevent any remaining scheduled stages from changing the simulator."""

        self._stopped = True

    @staticmethod
    def _all_devices_connected(snapshot: ServiceSnapshot) -> bool:
        return (
            snapshot.vehicle_connected
            and snapshot.lidar_connected
            and snapshot.controller_connected
        )

    def _connected(self) -> None:
        self._announce("connected")
        self.root.after(850, self._arm)

    def _arm(self) -> None:
        if self._stopped:
            return
        self.service.arm()
        self._wait_for(
            lambda: self.service.snapshot().phase == "armed",
            self._drive,
            timeout_ms=self.TRANSITION_TIMEOUT_MS,
        )

    def _drive(self) -> None:
        if self._stopped:
            return
        speed = 32
        steering = -18
        self.window.speed_value.set(speed)
        self.window.speed_label.config(text=f"{speed}%")
        self.window.steering_value.set(steering)
        self.window.steering_label.config(text=f"{steering}%")
        self.service.set_speed(speed)
        self.service.set_steering(steering)
        self._wait_for(
            lambda: self.service.snapshot().phase == "driving",
            self._driving,
            timeout_ms=self.TRANSITION_TIMEOUT_MS,
        )

    def _driving(self) -> None:
        self._announce("driving")
        self.root.after(1_700, self._show_lidar)

    def _show_lidar(self) -> None:
        if self._stopped:
            return
        self._select_tab(self.window.lidar_tab)
        self.window.simulated_distance.set(180.0)
        self.service.set_simulated_obstacle(180.0)
        self._announce("lidar_monitoring")
        self.root.after(1_000, lambda: self._move_obstacle(0))

    def _move_obstacle(self, index: int) -> None:
        if self._stopped:
            return
        distances = (140.0, 105.0, 72.0, 35.0)
        if index >= len(distances):
            self._wait_for(
                lambda: self.service.snapshot().estop_active,
                self._assisted_stop,
                timeout_ms=self.TRANSITION_TIMEOUT_MS,
            )
            return
        distance = distances[index]
        self.window.simulated_distance.set(distance)
        self.service.set_simulated_obstacle(distance)
        self._announce(f"obstacle_{round(distance)}cm")
        self.root.after(450, lambda: self._move_obstacle(index + 1))

    def _assisted_stop(self) -> None:
        self._announce("lidar_assisted_stop")
        self.root.after(1_400, self._show_diagnostics)

    def _show_diagnostics(self) -> None:
        if self._stopped:
            return
        self._select_tab(self.window.log_tab)
        self._announce("diagnostics")
        self.root.after(1_700, self._recover)

    def _recover(self) -> None:
        if self._stopped:
            return
        self.service.set_simulated_obstacle(180.0)
        self.service.reset_emergency_stop()
        self.window.speed_value.set(0)
        self.window.speed_label.config(text="0%")
        self.window.steering_value.set(0)
        self.window.steering_label.config(text="0%")
        self.window.simulated_distance.set(180.0)
        self._select_tab(self.window.control_tab)
        self._wait_for(
            lambda: self.service.snapshot().phase == "safe_connected",
            self._ready,
            timeout_ms=self.TRANSITION_TIMEOUT_MS,
        )

    def _ready(self) -> None:
        self._announce("ready")
        self.root.after(1_200, self._finish)

    def _finish(self) -> None:
        if self._stopped or self._finished:
            return
        self._finished = True
        self._announce("complete")
        if self.on_complete is not None:
            self.on_complete()

    def _wait_for(
        self,
        predicate: Callable[[], bool],
        on_ready: Callable[[], None],
        *,
        timeout_ms: int,
    ) -> None:
        deadline = time.monotonic() + timeout_ms / 1_000

        def poll() -> None:
            if self._stopped:
                return
            if predicate():
                on_ready()
                return
            if time.monotonic() >= deadline:
                self._stopped = True
                self._announce("timeout")
                if self.on_complete is not None:
                    self.on_complete()
                return
            self.root.after(self.POLL_MS, poll)

        self.root.after(self.POLL_MS, poll)

    def _announce(self, stage: str) -> None:
        if self.on_stage is not None:
            self.on_stage(stage)

    def _select_tab(self, tab: tk.Widget) -> None:
        cast(Any, self.window.notebook).select(tab)
