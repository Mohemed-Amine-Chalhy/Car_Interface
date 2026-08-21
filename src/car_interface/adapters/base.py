"""Interfaces shared by real and simulated devices.

The rest of the application depends on these small protocols instead of concrete
serial, Lidar, or controller libraries.  That keeps imports side-effect free and
makes every safety path testable without physical hardware.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Protocol, runtime_checkable


class AdapterError(RuntimeError):
    """Base error raised by a device adapter."""


class ConnectionFailed(AdapterError):
    """A requested device connection could not be established."""


class TransportDisconnected(AdapterError):
    """A transport operation was attempted after the connection was lost."""


@dataclass(frozen=True, slots=True)
class ScanPoint:
    """One normalized Lidar measurement."""

    angle_degrees: float
    distance_cm: float
    quality: int = 0


@dataclass(frozen=True, slots=True)
class ControllerSnapshot:
    """Normalized controller state, independent of controller brand."""

    connected: bool
    steering: float = 0.0
    throttle: float = 0.0
    brake_pressed: bool = False
    direction: str = "forward"

    def __post_init__(self) -> None:
        if not isinstance(self.connected, bool):
            raise TypeError("connected must be a boolean")
        if not isinstance(self.brake_pressed, bool):
            raise TypeError("brake_pressed must be a boolean")
        for name, value, minimum, maximum in (
            ("steering", self.steering, -1.0, 1.0),
            ("throttle", self.throttle, 0.0, 1.0),
        ):
            if isinstance(value, bool) or not isinstance(value, int | float):
                raise TypeError(f"{name} must be a number")
            if not isfinite(value) or not minimum <= value <= maximum:
                raise ValueError(f"{name} must be finite and between {minimum:g} and {maximum:g}")
        if self.direction not in {"forward", "reverse"}:
            raise ValueError("direction must be 'forward' or 'reverse'")


@runtime_checkable
class VehicleTransport(Protocol):
    """Exclusive low-level link to the ESP32 firmware."""

    @property
    def is_connected(self) -> bool:
        """Return whether the link is currently usable."""

    @property
    def description(self) -> str:
        """Return a non-secret human-readable device description."""

    def connect(self) -> None:
        """Open the transport or raise :class:`ConnectionFailed`."""

    def transact(self, frame: str, timeout_seconds: float) -> str | None:
        """Write one frame and return one response line, if any."""

    def disconnect(self) -> None:
        """Close the transport. Implementations must be idempotent."""


@runtime_checkable
class LidarSource(Protocol):
    """Normalized source of recent Lidar scans."""

    @property
    def is_connected(self) -> bool:
        """Return whether the Lidar worker is healthy."""

    @property
    def description(self) -> str:
        """Return a human-readable device description."""

    def connect(self) -> None:
        """Connect and start scanning."""

    def latest_scan(self) -> tuple[tuple[ScanPoint, ...], float] | None:
        """Return points and their ``monotonic()`` timestamp."""

    def disconnect(self) -> None:
        """Stop and disconnect. Implementations must be idempotent."""


@runtime_checkable
class ControllerSource(Protocol):
    """Normalized game-controller input."""

    @property
    def is_connected(self) -> bool:
        """Return whether the controller is connected."""

    @property
    def description(self) -> str:
        """Return the detected controller name."""

    def connect(self) -> None:
        """Connect to the configured controller."""

    def poll(self) -> ControllerSnapshot:
        """Read the current state without blocking."""

    def disconnect(self) -> None:
        """Release controller resources."""
