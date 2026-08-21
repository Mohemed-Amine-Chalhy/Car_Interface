"""Pure Lidar calculations shared by live and simulated operation."""

from __future__ import annotations

import math
from dataclasses import dataclass

from car_interface.adapters.base import ScanPoint


@dataclass(frozen=True, slots=True)
class LidarAssessment:
    points: tuple[ScanPoint, ...]
    closest_in_path_cm: float | None
    closest_angle_degrees: float | None
    stale: bool


def assess_scan(
    points: tuple[ScanPoint, ...],
    *,
    captured_at: float,
    now: float,
    vehicle_width_cm: float,
    stale_after_seconds: float,
    view_angle_degrees: float = 180.0,
) -> LidarAssessment:
    """Find the closest valid point inside the vehicle's projected path."""

    if not math.isfinite(now) or now < 0:
        raise ValueError("now must be finite and non-negative")
    if not math.isfinite(stale_after_seconds) or stale_after_seconds <= 0:
        raise ValueError("stale_after_seconds must be finite and positive")
    if not math.isfinite(vehicle_width_cm) or vehicle_width_cm <= 0:
        raise ValueError("vehicle_width_cm must be finite and positive")
    if not math.isfinite(view_angle_degrees) or not 0 < view_angle_degrees <= 360:
        raise ValueError("view_angle_degrees must be finite and between 0 and 360")

    stale = (
        not math.isfinite(captured_at)
        or captured_at < 0
        or captured_at > now
        or now - captured_at > stale_after_seconds
    )
    closest_distance: float | None = None
    closest_angle: float | None = None
    half_width = vehicle_width_cm / 2.0
    half_view = view_angle_degrees / 2.0

    for point in points:
        if not math.isfinite(point.distance_cm) or point.distance_cm <= 0:
            continue
        if not -half_view <= point.angle_degrees <= half_view:
            continue
        lateral_distance = abs(point.distance_cm * math.sin(math.radians(point.angle_degrees)))
        if lateral_distance > half_width:
            continue
        if closest_distance is None or point.distance_cm < closest_distance:
            closest_distance = point.distance_cm
            closest_angle = point.angle_degrees

    return LidarAssessment(
        points=points,
        closest_in_path_cm=closest_distance,
        closest_angle_degrees=closest_angle,
        stale=stale,
    )
