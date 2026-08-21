"""Typed vehicle commands and the versioned serial framing contract."""

from __future__ import annotations

import re
from dataclasses import dataclass
from enum import IntEnum, StrEnum
from math import isfinite
from typing import Final

from .state import ControlMode, FaultCode

MIN_PERCENT: Final = -100
MAX_PERCENT: Final = 100
MIN_SEQUENCE: Final = 0
MAX_SEQUENCE: Final = (2**31) - 1


class CommandValidationError(ValueError):
    """Raised when construction of a command would violate the contract."""


class ProtocolError(ValueError):
    """Raised for malformed, unsupported, corrupt, or unexpected frames."""


class CommandType(StrEnum):
    SET_SPEED = "SPD"
    SET_STEERING = "STR"
    SET_BRAKE = "BRK"
    SET_MODE = "MOD"
    SET_ARMED = "ARM"
    HEARTBEAT = "HBT"
    EMERGENCY_STOP = "EST"
    RESET = "RST"


class CommandPriority(IntEnum):
    """Lower values must be dispatched first."""

    EMERGENCY = 0
    SAFETY = 10
    CONTROL = 50
    BACKGROUND = 100


type CommandValue = int | bool | ControlMode | None


@dataclass(frozen=True, slots=True)
class ControlCommand:
    """One validated command to the actuator controller."""

    kind: CommandType
    value: CommandValue = None

    def __post_init__(self) -> None:
        if self.kind in {CommandType.SET_SPEED, CommandType.SET_STEERING}:
            if isinstance(self.value, bool) or not isinstance(self.value, int):
                raise CommandValidationError(f"{self.kind.name} requires an integer")
            if not MIN_PERCENT <= self.value <= MAX_PERCENT:
                raise CommandValidationError(
                    f"{self.kind.name} must be between {MIN_PERCENT} and {MAX_PERCENT}"
                )
            return

        if self.kind in {CommandType.SET_BRAKE, CommandType.SET_ARMED}:
            if not isinstance(self.value, bool):
                raise CommandValidationError(f"{self.kind.name} requires a boolean")
            return

        if self.kind is CommandType.SET_MODE:
            if not isinstance(self.value, ControlMode):
                raise CommandValidationError("SET_MODE requires a ControlMode")
            return

        if self.value is not None:
            raise CommandValidationError(f"{self.kind.name} does not accept a value")

    @classmethod
    def set_speed(cls, percent: int) -> ControlCommand:
        return cls(CommandType.SET_SPEED, percent)

    @classmethod
    def set_steering(cls, percent: int) -> ControlCommand:
        return cls(CommandType.SET_STEERING, percent)

    @classmethod
    def set_brake(cls, engaged: bool) -> ControlCommand:
        return cls(CommandType.SET_BRAKE, engaged)

    @classmethod
    def set_mode(cls, mode: ControlMode) -> ControlCommand:
        return cls(CommandType.SET_MODE, mode)

    @classmethod
    def set_armed(cls, armed: bool) -> ControlCommand:
        return cls(CommandType.SET_ARMED, armed)

    @classmethod
    def heartbeat(cls) -> ControlCommand:
        return cls(CommandType.HEARTBEAT)

    @classmethod
    def emergency_stop(cls) -> ControlCommand:
        return cls(CommandType.EMERGENCY_STOP)

    @classmethod
    def reset(cls) -> ControlCommand:
        return cls(CommandType.RESET)

    @property
    def priority(self) -> CommandPriority:
        if self.kind is CommandType.EMERGENCY_STOP:
            return CommandPriority.EMERGENCY
        if (
            (self.kind is CommandType.SET_SPEED and self.value == 0)
            or (self.kind is CommandType.SET_BRAKE and self.value is True)
            or (self.kind is CommandType.SET_ARMED and self.value is False)
            or self.kind is CommandType.RESET
        ):
            return CommandPriority.SAFETY
        if self.kind is CommandType.HEARTBEAT:
            return CommandPriority.BACKGROUND
        return CommandPriority.CONTROL

    @property
    def is_motion(self) -> bool:
        """Whether this can initiate or alter physical motion."""

        return self.kind in {CommandType.SET_SPEED, CommandType.SET_STEERING} and (
            self.kind is CommandType.SET_STEERING or self.value != 0
        )

    @property
    def requires_freshness(self) -> bool:
        """Whether delayed delivery could create or materially alter motion."""

        return self.is_motion or (
            (self.kind is CommandType.SET_BRAKE and self.value is False)
            or (self.kind is CommandType.SET_ARMED and self.value is True)
        )

    @property
    def requires_acknowledgement(self) -> bool:
        """All protocol-v1 commands require a matching ACK or NACK."""

        return True


class ResponseKind(StrEnum):
    ACK = "ACK"
    NACK = "NACK"


@dataclass(frozen=True, slots=True)
class ProtocolResponse:
    kind: ResponseKind
    sequence: int
    fault_code: str | None = None

    def __post_init__(self) -> None:
        _validate_sequence(self.sequence)
        if self.kind is ResponseKind.ACK and self.fault_code is not None:
            raise ValueError("ACK responses cannot contain a fault code")
        if self.kind is ResponseKind.NACK and (
            self.fault_code is None or not _is_protocol_token(self.fault_code)
        ):
            raise ValueError("NACK responses require a valid fault code")


@dataclass(frozen=True, slots=True)
class DecodedCommand:
    sequence: int
    command: ControlCommand

    def __post_init__(self) -> None:
        _validate_sequence(self.sequence)


@dataclass(frozen=True, slots=True)
class ProtocolContract:
    """Wire-level timing and framing requirements for protocol version 1."""

    protocol_name: str = "CAR"
    version: int = 1
    max_frame_bytes: int = 128
    ack_timeout_seconds: float = 0.2
    heartbeat_interval_seconds: float = 0.1
    watchdog_timeout_seconds: float = 0.5

    def __post_init__(self) -> None:
        if not _is_protocol_token(self.protocol_name):
            raise ValueError("protocol_name must be an uppercase protocol token")
        if self.version <= 0:
            raise ValueError("version must be positive")
        if self.max_frame_bytes < 32:
            raise ValueError("max_frame_bytes must be at least 32")
        for field_name, value in (
            ("ack_timeout_seconds", self.ack_timeout_seconds),
            ("heartbeat_interval_seconds", self.heartbeat_interval_seconds),
            ("watchdog_timeout_seconds", self.watchdog_timeout_seconds),
        ):
            if not isfinite(value) or value <= 0:
                raise ValueError(f"{field_name} must be finite and positive")
        if self.watchdog_timeout_seconds <= (
            self.heartbeat_interval_seconds + self.ack_timeout_seconds
        ):
            raise ValueError(
                "watchdog_timeout_seconds must exceed heartbeat interval plus ACK timeout"
            )


_TOKEN_RE: Final = re.compile(r"^[A-Z][A-Z0-9_]{0,31}$")


def _is_protocol_token(value: str) -> bool:
    return _TOKEN_RE.fullmatch(value) is not None


def _validate_sequence(sequence: int) -> None:
    if isinstance(sequence, bool) or not isinstance(sequence, int):
        raise TypeError("sequence must be an integer")
    if not MIN_SEQUENCE <= sequence <= MAX_SEQUENCE:
        raise ValueError(f"sequence must be between {MIN_SEQUENCE} and {MAX_SEQUENCE}")


def crc16_ccitt(data: bytes) -> int:
    """Return CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)."""

    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class ProtocolCodec:
    """Encode and parse newline-delimited protocol-v1 ASCII frames.

    ``encode_*`` methods return ``str`` frames including their trailing newline.
    Parsers accept either ``str`` or ASCII ``bytes`` and tolerate an omitted
    final newline, which is convenient for transports that split lines first.
    """

    def __init__(self, contract: ProtocolContract | None = None) -> None:
        self.contract = contract or ProtocolContract()

    def encode_command(self, command: ControlCommand, sequence: int) -> str:
        _validate_sequence(sequence)
        fields = [
            self.contract.protocol_name,
            str(self.contract.version),
            "CMD",
            str(sequence),
            command.kind.value,
        ]
        encoded_value = self._encode_value(command)
        if encoded_value is not None:
            fields.append(encoded_value)
        return self._frame(fields)

    def encode_command_bytes(self, command: ControlCommand, sequence: int) -> bytes:
        return self.encode_command(command, sequence).encode("ascii")

    def parse_command(self, line: str | bytes) -> DecodedCommand:
        fields = self._parse_frame(line)
        if len(fields) not in {5, 6} or fields[2] != "CMD":
            raise ProtocolError("expected a command frame")
        sequence = self._parse_sequence(fields[3])
        try:
            kind = CommandType(fields[4])
        except ValueError as exc:
            raise ProtocolError(f"unknown command opcode: {fields[4]!r}") from exc

        values = fields[5:]
        if kind in {CommandType.SET_SPEED, CommandType.SET_STEERING}:
            if len(values) != 1 or not _is_canonical_integer(values[0]):
                raise ProtocolError(f"{kind.value} requires one canonical integer")
            try:
                command = ControlCommand(kind, int(values[0]))
            except CommandValidationError as exc:
                raise ProtocolError(str(exc)) from exc
        elif kind in {CommandType.SET_BRAKE, CommandType.SET_ARMED}:
            if values not in (["0"], ["1"]):
                raise ProtocolError(f"{kind.value} requires 0 or 1")
            command = ControlCommand(kind, values[0] == "1")
        elif kind is CommandType.SET_MODE:
            if len(values) != 1:
                raise ProtocolError("MOD requires one mode")
            try:
                command = ControlCommand(kind, ControlMode(values[0].lower()))
            except (ValueError, CommandValidationError) as exc:
                raise ProtocolError(f"unknown control mode: {values[0]!r}") from exc
        else:
            if values:
                raise ProtocolError(f"{kind.value} does not accept a value")
            command = ControlCommand(kind)
        return DecodedCommand(sequence=sequence, command=command)

    def encode_ack(self, sequence: int) -> str:
        _validate_sequence(sequence)
        return self._frame(
            [self.contract.protocol_name, str(self.contract.version), "ACK", str(sequence)]
        )

    def encode_nack(self, sequence: int, fault_code: FaultCode | str) -> str:
        _validate_sequence(sequence)
        code = fault_code.value.upper() if isinstance(fault_code, FaultCode) else fault_code
        if not _is_protocol_token(code):
            raise ValueError("fault_code must be an uppercase protocol token")
        return self._frame(
            [
                self.contract.protocol_name,
                str(self.contract.version),
                "NACK",
                str(sequence),
                code,
            ]
        )

    def parse_response(
        self, line: str | bytes, expected_sequence: int | None = None
    ) -> ProtocolResponse:
        fields = self._parse_frame(line)
        if len(fields) not in {4, 5}:
            raise ProtocolError("expected an ACK or NACK frame")
        try:
            kind = ResponseKind(fields[2])
        except ValueError as exc:
            raise ProtocolError("expected an ACK or NACK frame") from exc
        sequence = self._parse_sequence(fields[3])
        if expected_sequence is not None:
            _validate_sequence(expected_sequence)
            if sequence != expected_sequence:
                raise ProtocolError(
                    f"unexpected response sequence {sequence}; expected {expected_sequence}"
                )

        if kind is ResponseKind.ACK:
            if len(fields) != 4:
                raise ProtocolError("ACK must not contain a fault code")
            return ProtocolResponse(kind=kind, sequence=sequence)

        if len(fields) != 5 or not _is_protocol_token(fields[4]):
            raise ProtocolError("NACK requires a valid fault code")
        return ProtocolResponse(kind=kind, sequence=sequence, fault_code=fields[4])

    def _frame(self, fields: list[str]) -> str:
        body = ",".join(fields)
        checksum = crc16_ccitt(body.encode("ascii"))
        frame = f"!{body}*{checksum:04X}\n"
        if len(frame.encode("ascii")) > self.contract.max_frame_bytes:
            raise ProtocolError("encoded frame exceeds maximum length")
        return frame

    def _parse_frame(self, line: str | bytes) -> list[str]:
        if isinstance(line, bytes):
            try:
                text = line.decode("ascii")
            except UnicodeDecodeError as exc:
                raise ProtocolError("frame must contain ASCII only") from exc
        elif isinstance(line, str):
            text = line
            try:
                text.encode("ascii")
            except UnicodeEncodeError as exc:
                raise ProtocolError("frame must contain ASCII only") from exc
        else:
            raise TypeError("line must be str or bytes")

        if len(text.encode("ascii")) > self.contract.max_frame_bytes:
            raise ProtocolError("frame exceeds maximum length")
        text = text.removesuffix("\n")
        if "\r" in text or "\n" in text:
            raise ProtocolError("frame contains an embedded line ending")
        if not text.startswith("!") or text.count("*") != 1:
            raise ProtocolError("frame delimiters are invalid")

        body, checksum_text = text[1:].split("*", maxsplit=1)
        if not re.fullmatch(r"[0-9A-F]{4}", checksum_text):
            raise ProtocolError("checksum must be four uppercase hexadecimal digits")
        actual_checksum = crc16_ccitt(body.encode("ascii"))
        if int(checksum_text, 16) != actual_checksum:
            raise ProtocolError("checksum mismatch")

        fields = body.split(",")
        if len(fields) < 4 or any(field == "" for field in fields):
            raise ProtocolError("frame fields are incomplete")
        if fields[0] != self.contract.protocol_name:
            raise ProtocolError(f"unsupported protocol name: {fields[0]!r}")
        if fields[1] != str(self.contract.version):
            raise ProtocolError(f"unsupported protocol version: {fields[1]!r}")
        return fields

    @staticmethod
    def _encode_value(command: ControlCommand) -> str | None:
        if command.value is None:
            return None
        if isinstance(command.value, ControlMode):
            return command.value.value.upper()
        if isinstance(command.value, bool):
            return "1" if command.value else "0"
        return str(command.value)

    @staticmethod
    def _parse_sequence(value: str) -> int:
        if (
            not value.isascii()
            or not value.isdecimal()
            or (len(value) > 1 and value.startswith("0"))
        ):
            raise ProtocolError("sequence must be a canonical non-negative integer")
        sequence = int(value)
        try:
            _validate_sequence(sequence)
        except (TypeError, ValueError) as exc:
            raise ProtocolError(str(exc)) from exc
        return sequence


def _is_canonical_integer(value: str) -> bool:
    if value == "0":
        return True
    if value.startswith("-"):
        magnitude = value[1:]
        return (
            bool(magnitude)
            and magnitude.isascii()
            and magnitude.isdecimal()
            and not magnitude.startswith("0")
        )
    return value.isascii() and value.isdecimal() and not value.startswith("0")
