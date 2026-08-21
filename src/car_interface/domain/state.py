"""Immutable domain state for vehicle control and safety.

The types in this module deliberately know nothing about serial ports, GUI
toolkits, clocks, or threads.  Timestamps are monotonic seconds supplied by the
caller so the safety model is deterministic and straightforward to test.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from math import isfinite


class SafetyPhase(StrEnum):
    """Top-level, operator-visible safety phase."""

    DISCONNECTED = "disconnected"
    SAFE_CONNECTED = "safe_connected"
    ARMED = "armed"
    DRIVING = "driving"
    BRAKING = "braking"
    EMERGENCY_STOP = "emergency_stop"
    FAULT = "fault"


class ControlMode(StrEnum):
    """Source of vehicle motion requests."""

    MANUAL = "manual"
    AUTONOMOUS = "autonomous"


class DeviceKind(StrEnum):
    """Devices known to the safety layer."""

    ACTUATOR = "actuator"
    CONTROLLER = "controller"
    LIDAR = "lidar"


class DeviceConnection(StrEnum):
    """Health of a device from the application's point of view."""

    DISCONNECTED = "disconnected"
    CONNECTED = "connected"
    STALE = "stale"
    FAILED = "failed"


class FaultCode(StrEnum):
    """Stable fault identifiers suitable for logs and operator diagnostics."""

    ACTUATOR_LOST = "actuator_lost"
    CONTROLLER_LOST = "controller_lost"
    SENSOR_LOST = "sensor_lost"
    STALE_DATA = "stale_data"
    COMMUNICATION_ERROR = "communication_error"
    PROTOCOL_ERROR = "protocol_error"
    WATCHDOG_TIMEOUT = "watchdog_timeout"
    INTERNAL_ERROR = "internal_error"


def _validate_timestamp(value: float, field_name: str) -> None:
    if not isfinite(value) or value < 0:
        raise ValueError(f"{field_name} must be a finite, non-negative number")


@dataclass(frozen=True, slots=True)
class Fault:
    """A latched safety fault.

    ``occurred_at_seconds`` is measured using the caller's monotonic clock.
    """

    code: FaultCode
    detail: str
    occurred_at_seconds: float
    source: DeviceKind | None = None

    def __post_init__(self) -> None:
        if not self.detail.strip():
            raise ValueError("fault detail must not be empty")
        _validate_timestamp(self.occurred_at_seconds, "occurred_at_seconds")


@dataclass(frozen=True, slots=True)
class DeviceStatus:
    """Last known health and heartbeat for one device."""

    device: DeviceKind
    connection: DeviceConnection = DeviceConnection.DISCONNECTED
    required: bool = False
    last_seen_seconds: float | None = None
    detail: str | None = None

    def __post_init__(self) -> None:
        if self.last_seen_seconds is not None:
            _validate_timestamp(self.last_seen_seconds, "last_seen_seconds")
        if self.detail is not None and not self.detail.strip():
            raise ValueError("device detail must be non-empty when provided")

    def is_fresh(self, *, now: float, timeout_seconds: float | None) -> bool:
        """Return whether this device is connected and within its timeout."""

        _validate_timestamp(now, "now")
        if self.connection is not DeviceConnection.CONNECTED:
            return False
        if timeout_seconds is None:
            return True
        if timeout_seconds <= 0 or not isfinite(timeout_seconds):
            raise ValueError("timeout_seconds must be finite and positive")
        return self.last_seen_seconds is not None and (
            now - self.last_seen_seconds <= timeout_seconds
        )


@dataclass(frozen=True, slots=True)
class ControlState:
    """Complete immutable snapshot of safety-relevant application state."""

    phase: SafetyPhase
    mode: ControlMode
    speed_percent: int
    steering_percent: int
    brake_engaged: bool
    emergency_stop_latched: bool
    devices: tuple[DeviceStatus, ...]
    active_fault: Fault | None = None
    revision: int = 0
    last_event_seconds: float | None = None

    def __post_init__(self) -> None:
        for name, value in (
            ("speed_percent", self.speed_percent),
            ("steering_percent", self.steering_percent),
        ):
            if isinstance(value, bool) or not isinstance(value, int):
                raise TypeError(f"{name} must be an integer")
            if not -100 <= value <= 100:
                raise ValueError(f"{name} must be between -100 and 100")

        if self.revision < 0:
            raise ValueError("revision must be non-negative")
        if self.last_event_seconds is not None:
            _validate_timestamp(self.last_event_seconds, "last_event_seconds")

        device_names = [status.device for status in self.devices]
        if len(device_names) != len(set(device_names)):
            raise ValueError("devices must contain at most one status per device")

        if self.phase is SafetyPhase.DRIVING and self.brake_engaged:
            raise ValueError("the brake cannot be engaged while driving")
        if self.phase is not SafetyPhase.DRIVING and self.speed_percent != 0:
            raise ValueError("speed must be zero outside the driving phase")
        if (
            self.phase
            in {
                SafetyPhase.DISCONNECTED,
                SafetyPhase.SAFE_CONNECTED,
                SafetyPhase.ARMED,
                SafetyPhase.BRAKING,
                SafetyPhase.EMERGENCY_STOP,
                SafetyPhase.FAULT,
            }
            and not self.brake_engaged
        ):
            raise ValueError(f"brake must be engaged in {self.phase.value}")
        if self.phase is SafetyPhase.EMERGENCY_STOP and not self.emergency_stop_latched:
            raise ValueError("emergency-stop phase must be latched")
        if self.phase is SafetyPhase.FAULT and self.active_fault is None:
            raise ValueError("fault phase requires an active fault")

    def device_status(self, device: DeviceKind) -> DeviceStatus:
        """Return one device status, raising ``KeyError`` when it is unknown."""

        for status in self.devices:
            if status.device is device:
                return status
        raise KeyError(device)
