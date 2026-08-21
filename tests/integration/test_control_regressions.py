from __future__ import annotations

import threading
import time
from collections.abc import Callable

from car_interface.adapters.base import ControllerSnapshot, ScanPoint
from car_interface.adapters.simulated import (
    SimulatedControllerSource,
    SimulatedLidarSource,
    SimulatedVehicleTransport,
)
from car_interface.config import AppConfig
from car_interface.services.control import ControlService


def _wait_until(predicate: Callable[[], bool], timeout: float = 5.0) -> None:
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.005)
    assert predicate()


def _service(
    *,
    vehicle: SimulatedVehicleTransport | None = None,
    lidar: SimulatedLidarSource | None = None,
    controller: SimulatedControllerSource | None = None,
    config: AppConfig | None = None,
) -> tuple[
    ControlService,
    SimulatedVehicleTransport,
    SimulatedLidarSource,
    SimulatedControllerSource,
]:
    actual_vehicle = vehicle or SimulatedVehicleTransport()
    actual_lidar = lidar or SimulatedLidarSource()
    actual_controller = controller or SimulatedControllerSource()
    return (
        ControlService(
            config or AppConfig(),
            vehicle=actual_vehicle,
            lidar=actual_lidar,
            controller=actual_controller,
        ),
        actual_vehicle,
        actual_lidar,
        actual_controller,
    )


def test_held_throttle_prevents_arming() -> None:
    service, vehicle, _, controller = _service()
    try:
        service.connect()
        _wait_until(lambda: service.snapshot().phase == "safe_connected")
        controller.snapshot = ControllerSnapshot(connected=True, throttle=0.8)
        _wait_until(lambda: not service._controller_inputs_neutral)

        service.arm()

        assert service.snapshot().phase == "safe_connected"
        assert not vehicle.armed
        assert vehicle.speed_percent == 0
    finally:
        service.shutdown()


def test_obstacle_inside_stop_limit_prevents_arming() -> None:
    service, vehicle, _, _ = _service()
    try:
        service.connect()
        _wait_until(lambda: service.snapshot().phase == "safe_connected")
        service.set_simulated_obstacle(25)
        _wait_until(
            lambda: (
                service.snapshot().closest_obstacle_cm is not None
                and (service.snapshot().closest_obstacle_cm or 999) < 50
            )
        )

        service.arm()

        assert service.snapshot().phase == "safe_connected"
        assert not vehicle.armed
        assert vehicle.speed_percent == 0
        assert vehicle.braked
    finally:
        service.shutdown()


def test_braking_with_held_throttle_requires_neutral_and_explicit_rearm() -> None:
    service, vehicle, _, controller = _service()
    try:
        service.connect()
        _wait_until(lambda: service.snapshot().phase == "safe_connected")
        service.arm()
        _wait_until(lambda: vehicle.armed)
        service.set_speed(20)
        _wait_until(lambda: vehicle.speed_percent == 20)

        controller.snapshot = ControllerSnapshot(connected=True, throttle=0.7)
        _wait_until(lambda: not service._controller_inputs_neutral)
        service.set_brake(True)
        _wait_until(lambda: service.snapshot().phase == "braking" and vehicle.braked)

        service.arm()
        assert service.snapshot().phase == "braking"
        controller.snapshot = ControllerSnapshot(connected=True)
        _wait_until(
            lambda: service._controller_inputs_neutral and not service._resume_requires_neutral
        )
        assert service.snapshot().phase == "braking"
        assert vehicle.speed_percent == 0

        service.arm()
        _wait_until(lambda: service.snapshot().phase == "armed" and vehicle.armed)
        assert vehicle.speed_percent == 0
    finally:
        service.shutdown()


def test_controller_brake_release_keeps_brake_held_until_explicit_arm() -> None:
    service, vehicle, _, controller = _service()
    try:
        service.connect()
        _wait_until(lambda: service.snapshot().phase == "safe_connected")
        _wait_until(lambda: len(vehicle.frames) >= 5)
        service.arm()
        _wait_until(lambda: vehicle.armed)
        service.set_speed(20)
        _wait_until(lambda: vehicle.speed_percent == 20)

        controller.snapshot = ControllerSnapshot(connected=True, brake_pressed=True)
        _wait_until(lambda: service.snapshot().phase == "braking" and vehicle.braked)
        controller.snapshot = ControllerSnapshot(connected=True)
        _wait_until(
            lambda: service._controller_inputs_neutral and not service._resume_requires_neutral
        )
        time.sleep(0.08)

        assert service.snapshot().phase == "braking"
        assert vehicle.speed_percent == 0
        assert vehicle.braked

        service.arm()
        _wait_until(lambda: service.snapshot().phase == "armed" and vehicle.armed)
        assert vehicle.braked
    finally:
        service.shutdown()


class CachedTimestampLidar(SimulatedLidarSource):
    def __init__(self) -> None:
        super().__init__()
        self._cached_timestamp = 0.0

    def connect(self) -> None:
        super().connect()
        self._cached_timestamp = time.monotonic()

    def latest_scan(self) -> tuple[tuple[ScanPoint, ...], float] | None:
        if not self.is_connected:
            return None
        points = (
            ScanPoint(angle_degrees=-45.0, distance_cm=250.0, quality=15),
            ScanPoint(angle_degrees=45.0, distance_cm=250.0, quality=15),
        )
        return points, self._cached_timestamp


def test_cached_lidar_scan_timestamp_cannot_refresh_freshness() -> None:
    lidar = CachedTimestampLidar()
    config = AppConfig(lidar_stale_seconds=0.08)
    service, vehicle, _, _ = _service(lidar=lidar, config=config)
    try:
        service.connect()
        _wait_until(lambda: service.snapshot().phase == "safe_connected")
        _wait_until(lambda: service.snapshot().fault is not None)

        snapshot = service.snapshot()
        assert "lidar" in (snapshot.fault or "").lower()
        _wait_until(lambda: vehicle.estopped)
        assert vehicle.speed_percent == 0
        assert vehicle.braked
    finally:
        service.shutdown()


class GatedConnectTransport(SimulatedVehicleTransport):
    def __init__(self) -> None:
        super().__init__()
        self.connect_entered = threading.Event()
        self.release_connect = threading.Event()

    def connect(self) -> None:
        self.connect_entered.set()
        if not self.release_connect.wait(2.0):
            raise TimeoutError("test did not release vehicle connection")
        super().connect()


def test_connect_cancellation_cleans_up_without_starting_supervision() -> None:
    vehicle = GatedConnectTransport()
    service, _, lidar, controller = _service(vehicle=vehicle)
    try:
        service.connect()
        assert vehicle.connect_entered.wait(1.0)
        service.disconnect()
        vehicle.release_connect.set()
        _wait_until(lambda: not vehicle.is_connected)
        _wait_until(lambda: service.snapshot().phase == "disconnected")

        assert not lidar.is_connected
        assert not controller.is_connected
        assert vehicle.frames == ()
    finally:
        vehicle.release_connect.set()
        service.shutdown()
