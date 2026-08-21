"""Stable presentation models exposed by the control service."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class ServiceSnapshot:
    phase: str = "disconnected"
    mode: str = "manual"
    vehicle_connected: bool = False
    lidar_connected: bool = False
    controller_connected: bool = False
    speed_percent: int = 0
    steering_percent: int = 0
    direction: str = "forward"
    brake_active: bool = True
    estop_active: bool = False
    auto_stop_enabled: bool = True
    closest_obstacle_cm: float | None = None
    fault: str | None = None
