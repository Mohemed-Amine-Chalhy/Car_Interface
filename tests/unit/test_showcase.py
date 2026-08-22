from __future__ import annotations

from dataclasses import replace
from unittest.mock import Mock, patch

import pytest

from car_interface.services.models import ServiceSnapshot
from car_interface.ui.showcase import ShowcaseDirector


class FakeRoot:
    def __init__(self) -> None:
        self.callbacks: list[object] = []

    def after(self, _delay: int, callback) -> str:
        self.callbacks.append(callback)
        return "after-id"

    def run_all(self, limit: int = 100) -> None:
        steps = 0
        while self.callbacks:
            callback = self.callbacks.pop(0)
            callback()
            steps += 1
            if steps > limit:
                raise AssertionError("showcase callbacks did not terminate")


class FakeVariable:
    def __init__(self) -> None:
        self.value: int | float = 0

    def set(self, value: int | float) -> None:
        self.value = value


class FakeWindow:
    def __init__(self, *, simulation: bool = True) -> None:
        self.simulation = simulation
        self.notebook = Mock()
        self.control_tab = object()
        self.lidar_tab = object()
        self.log_tab = object()
        self.speed_value = FakeVariable()
        self.steering_value = FakeVariable()
        self.simulated_distance = FakeVariable()
        self.speed_label = Mock()
        self.steering_label = Mock()


class FakeService:
    def __init__(self) -> None:
        self.state = ServiceSnapshot()
        self.actions: list[tuple[str, int | float | None]] = []

    def snapshot(self) -> ServiceSnapshot:
        return self.state

    def connect(self) -> None:
        self.actions.append(("connect", None))
        self.state = replace(
            self.state,
            phase="safe_connected",
            vehicle_connected=True,
            lidar_connected=True,
            controller_connected=True,
        )

    def arm(self) -> None:
        self.actions.append(("arm", None))
        self.state = replace(self.state, phase="armed", brake_active=False)

    def set_speed(self, speed_percent: int) -> None:
        self.actions.append(("speed", speed_percent))
        self.state = replace(self.state, phase="driving", speed_percent=speed_percent)

    def set_steering(self, steering_percent: int) -> None:
        self.actions.append(("steering", steering_percent))
        self.state = replace(self.state, steering_percent=steering_percent)

    def set_simulated_obstacle(self, distance_cm: float) -> None:
        self.actions.append(("obstacle", distance_cm))
        self.state = replace(self.state, closest_obstacle_cm=distance_cm)
        if distance_cm < 50:
            self.state = replace(
                self.state,
                phase="emergency_stop",
                speed_percent=0,
                brake_active=True,
                estop_active=True,
            )

    def reset_emergency_stop(self) -> None:
        self.actions.append(("reset", None))
        self.state = replace(
            self.state,
            phase="safe_connected",
            brake_active=True,
            estop_active=False,
        )


def test_showcase_traverses_real_operator_story() -> None:
    root = FakeRoot()
    window = FakeWindow()
    service = FakeService()
    stages: list[str] = []
    complete = Mock()
    director = ShowcaseDirector(
        root,
        window,
        service,
        on_stage=stages.append,
        on_complete=complete,
    )

    director.start()
    director.start()
    root.run_all()

    assert stages == [
        "connecting",
        "connected",
        "driving",
        "lidar_monitoring",
        "obstacle_140cm",
        "obstacle_105cm",
        "obstacle_72cm",
        "obstacle_35cm",
        "lidar_assisted_stop",
        "diagnostics",
        "ready",
        "complete",
    ]
    assert service.actions == [
        ("connect", None),
        ("arm", None),
        ("speed", 32),
        ("steering", -18),
        ("obstacle", 180.0),
        ("obstacle", 140.0),
        ("obstacle", 105.0),
        ("obstacle", 72.0),
        ("obstacle", 35.0),
        ("obstacle", 180.0),
        ("reset", None),
    ]
    assert window.notebook.select.call_args_list == [
        ((window.control_tab,),),
        ((window.lidar_tab,),),
        ((window.log_tab,),),
        ((window.control_tab,),),
    ]
    assert window.speed_value.value == 0
    assert window.steering_value.value == 0
    assert window.simulated_distance.value == 180.0
    complete.assert_called_once_with()


def test_showcase_rejects_hardware_window() -> None:
    with pytest.raises(ValueError, match="only in simulation"):
        ShowcaseDirector(FakeRoot(), FakeWindow(simulation=False), FakeService())


def test_stopped_showcase_ignores_scheduled_work() -> None:
    root = FakeRoot()
    service = FakeService()
    director = ShowcaseDirector(root, FakeWindow(), service)

    director.stop()
    director.start()
    root.run_all()

    assert service.actions == []


def test_showcase_timeout_is_reported_and_completes() -> None:
    root = FakeRoot()
    service = FakeService()
    service.connect = Mock()
    stages: list[str] = []
    complete = Mock()
    director = ShowcaseDirector(
        root,
        FakeWindow(),
        service,
        on_stage=stages.append,
        on_complete=complete,
    )

    with patch("car_interface.ui.showcase.time.monotonic", side_effect=(0.0, 7.0)):
        director.start()
        root.run_all()

    assert stages == ["connecting", "timeout"]
    complete.assert_called_once_with()
