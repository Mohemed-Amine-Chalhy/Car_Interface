"""Explicit host-to-vehicle wire protocol profiles.

Protocol selection is configuration driven.  Nothing in this module probes a
serial device or attempts to infer which firmware is attached.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Protocol

from .commands import (
    CommandType,
    ControlCommand,
    ProtocolCodec,
    ProtocolError,
    ResponseKind,
)
from .state import ControlMode


@dataclass(frozen=True, slots=True)
class EncodedOperation:
    """One domain command represented as zero or more contiguous wire frames."""

    frames: tuple[str, ...]
    sequence: int | None


class CommandWireProtocol(Protocol):
    """Strategy used by the dispatcher to encode and verify one command."""

    @property
    def profile_id(self) -> str: ...

    @property
    def provides_acknowledgements(self) -> bool: ...

    @property
    def uses_sequences(self) -> bool: ...

    @property
    def supports_heartbeat(self) -> bool: ...

    @property
    def minimum_frame_interval_seconds(self) -> float: ...

    def encode(self, command: ControlCommand, sequence: int | None) -> EncodedOperation: ...

    def validate_response(self, line: str, expected_sequence: int | None) -> None: ...


class CarV1WireProtocol:
    """Adapter exposing the CRC-protected v1 codec through the profile API."""

    def __init__(self, codec: ProtocolCodec | None = None) -> None:
        self.codec = codec or ProtocolCodec()

    @property
    def profile_id(self) -> str:
        return "car_v1"

    @property
    def provides_acknowledgements(self) -> bool:
        return True

    @property
    def uses_sequences(self) -> bool:
        return True

    @property
    def supports_heartbeat(self) -> bool:
        return True

    @property
    def minimum_frame_interval_seconds(self) -> float:
        return 0.0

    def encode(self, command: ControlCommand, sequence: int | None) -> EncodedOperation:
        if sequence is None:
            raise ProtocolError("car_v1 requires a command sequence")
        frame = self.codec.encode_command(command, sequence).rstrip("\r\n")
        return EncodedOperation((frame,), sequence)

    def validate_response(self, line: str, expected_sequence: int | None) -> None:
        if expected_sequence is None:
            raise ProtocolError("car_v1 requires a response sequence")
        parsed = self.codec.parse_response(line, expected_sequence=expected_sequence)
        if parsed.kind is ResponseKind.NACK:
            raise ProtocolError(
                f"firmware rejected sequence {expected_sequence}: {parsed.fault_code}"
            )


@dataclass(frozen=True, slots=True)
class LegacyProtocolContract:
    """Calibration and pacing for the demonstrated school-car firmware."""

    steering_minimum: int = 200
    steering_center: int = 1_750
    steering_maximum: int = 2_900
    minimum_frame_interval_seconds: float = 0.05

    def __post_init__(self) -> None:
        for name, value in (
            ("steering_minimum", self.steering_minimum),
            ("steering_center", self.steering_center),
            ("steering_maximum", self.steering_maximum),
        ):
            if isinstance(value, bool) or not isinstance(value, int):
                raise TypeError(f"{name} must be an integer")
            if not 0 <= value <= 65_535:
                raise ValueError(f"{name} must be between 0 and 65535")
        if not self.steering_minimum < self.steering_center < self.steering_maximum:
            raise ValueError("legacy steering calibration must satisfy minimum < center < maximum")
        interval = self.minimum_frame_interval_seconds
        if isinstance(interval, bool) or not isinstance(interval, int | float):
            raise TypeError("minimum_frame_interval_seconds must be a number")
        if not isfinite(interval) or not 0 <= interval <= 1.0:
            raise ValueError("minimum_frame_interval_seconds must be finite and between 0 and 1")


class SchoolCarLegacyProtocol:
    """Write-only newline protocol used by the demonstrated school car.

    The firmware did not return command-correlated acknowledgements.  A
    successful dispatch therefore proves only that every line was written to
    the operating-system serial transport.
    """

    def __init__(self, contract: LegacyProtocolContract | None = None) -> None:
        self.contract = contract or LegacyProtocolContract()

    @property
    def profile_id(self) -> str:
        return "school_car_legacy_v0"

    @property
    def provides_acknowledgements(self) -> bool:
        return False

    @property
    def uses_sequences(self) -> bool:
        return False

    @property
    def supports_heartbeat(self) -> bool:
        return False

    @property
    def minimum_frame_interval_seconds(self) -> float:
        return self.contract.minimum_frame_interval_seconds

    def encode(self, command: ControlCommand, sequence: int | None) -> EncodedOperation:
        if sequence is not None:
            raise ProtocolError("school_car_legacy_v0 does not use command sequences")
        frames: tuple[str, ...]
        if command.kind is CommandType.SET_SPEED:
            speed = int(command.value) if command.value is not None else 0
            if speed > 0:
                frames = ("D F", f"V {speed}")
            elif speed < 0:
                frames = ("D R", f"V {abs(speed)}")
            else:
                frames = ("V 0",)
        elif command.kind is CommandType.SET_STEERING:
            percent = int(command.value) if command.value is not None else 0
            frames = (f"W {self.steering_raw(percent)}",)
        elif command.kind is CommandType.SET_BRAKE:
            frames = ("S 1" if bool(command.value) else "Q 1",)
        elif command.kind is CommandType.SET_MODE:
            frames = ("M" if command.value is ControlMode.MANUAL else "A",)
        elif command.kind is CommandType.SET_ARMED:
            # The demonstrated controller-driving path used automatic/remote
            # mode (A). Return to manual mode only after asserting the brake.
            frames = ("A",) if bool(command.value) else ("S 1", "M")
        elif command.kind is CommandType.HEARTBEAT:
            # No legacy heartbeat opcode exists. The service suppresses these,
            # but accepting one here keeps direct dispatcher use deterministic.
            frames = ()
        elif command.kind in {CommandType.EMERGENCY_STOP, CommandType.RESET}:
            frames = ("V 0", "S 1")
        else:  # pragma: no cover - enum exhaustiveness guard
            raise ProtocolError(f"unsupported legacy command: {command.kind.value}")
        return EncodedOperation(frames, None)

    def steering_raw(self, percent: int) -> int:
        """Map normalized steering through the calibrated center piecewise."""

        if isinstance(percent, bool) or not isinstance(percent, int):
            raise TypeError("steering percent must be an integer")
        if not -100 <= percent <= 100:
            raise ValueError("steering percent must be between -100 and 100")
        if percent < 0:
            span = self.contract.steering_center - self.contract.steering_minimum
            return self.contract.steering_center + _round_half_away_from_zero(span * percent / 100)
        span = self.contract.steering_maximum - self.contract.steering_center
        return self.contract.steering_center + _round_half_away_from_zero(span * percent / 100)

    def validate_response(self, line: str, expected_sequence: int | None) -> None:
        del line, expected_sequence
        raise ProtocolError("school_car_legacy_v0 does not provide acknowledgements")


def _round_half_away_from_zero(value: float) -> int:
    return int(value + 0.5) if value >= 0 else int(value - 0.5)
