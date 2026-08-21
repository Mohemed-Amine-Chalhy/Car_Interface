"""Deterministic adapters used by development, tests, and demonstrations."""

from __future__ import annotations

import math
import threading
import time

from car_interface.domain import (
    CommandType,
    ControlCommand,
    ProtocolCodec,
    ProtocolError,
    ResponseKind,
)

from .base import ControllerSnapshot, ScanPoint, TransportDisconnected


class SimulatedVehicleTransport:
    """In-memory ESP32 transport with observable safe-state behavior."""

    def __init__(self, watchdog_timeout_seconds: float = 0.5) -> None:
        if not math.isfinite(watchdog_timeout_seconds) or watchdog_timeout_seconds <= 0:
            raise ValueError("watchdog timeout must be finite and positive")
        self._connected = False
        self._lock = threading.Lock()
        self._frames: list[str] = []
        self._speed_percent = 0
        self._steering = 0
        self._braked = True
        self._estopped = False
        self._armed = False
        self._watchdog_tripped = False
        self._watchdog_timeout = watchdog_timeout_seconds
        self._last_valid_frame = 0.0
        self._watchdog_stop = threading.Event()
        self._watchdog_thread: threading.Thread | None = None
        self._next_sequence = 0
        self._last_sequence: int | None = None
        self._last_frame: str | None = None
        self._last_response: str | None = None
        self.fail_next_transaction = False
        self._codec = ProtocolCodec()

    @property
    def is_connected(self) -> bool:
        with self._lock:
            return self._connected

    @property
    def description(self) -> str:
        return "simulation:esp32"

    @property
    def frames(self) -> tuple[str, ...]:
        with self._lock:
            return tuple(self._frames)

    @property
    def speed_percent(self) -> int:
        with self._lock:
            return self._speed_percent

    @property
    def steering(self) -> int:
        with self._lock:
            return self._steering

    @property
    def braked(self) -> bool:
        with self._lock:
            return self._braked

    @property
    def estopped(self) -> bool:
        with self._lock:
            return self._estopped

    @property
    def armed(self) -> bool:
        with self._lock:
            return self._armed

    @property
    def watchdog_tripped(self) -> bool:
        with self._lock:
            return self._watchdog_tripped

    def connect(self) -> None:
        with self._lock:
            if self._connected:
                return
            self._connected = True
            self._speed_percent = 0
            self._steering = 0
            self._braked = True
            self._estopped = False
            self._armed = False
            self._watchdog_tripped = False
            self._next_sequence = 0
            self._last_sequence = None
            self._last_frame = None
            self._last_response = None
            self._last_valid_frame = time.monotonic()
            self._watchdog_stop.clear()
            self._watchdog_thread = threading.Thread(
                target=self._watchdog_loop,
                name="simulated-firmware-watchdog",
                daemon=True,
            )
            self._watchdog_thread.start()

    def transact(self, frame: str, timeout_seconds: float) -> str | None:
        if not math.isfinite(timeout_seconds) or timeout_seconds < 0:
            raise ValueError("timeout_seconds must be finite and non-negative")
        with self._lock:
            if not self._connected:
                raise TransportDisconnected("simulated ESP32 is disconnected")
            if self.fail_next_transaction:
                self.fail_next_transaction = False
                self._connected = False
                raise TransportDisconnected("injected simulated transport failure")
            self._frames.append(frame)
            try:
                decoded = self._codec.parse_command(frame)
            except ProtocolError:
                return None
            canonical_frame = frame.removesuffix("\n")
            if decoded.sequence == self._last_sequence:
                if canonical_frame != self._last_frame:
                    return self._codec.encode_nack(decoded.sequence, "BAD_SEQUENCE").strip()
                if self._last_response is not None:
                    parsed = self._codec.parse_response(self._last_response)
                    if parsed.kind is ResponseKind.ACK:
                        self._last_valid_frame = time.monotonic()
                return self._last_response
            if decoded.sequence != self._next_sequence:
                return self._codec.encode_nack(decoded.sequence, "BAD_SEQUENCE").strip()
            fault_code = self._apply_command_unlocked(decoded.command)
            if fault_code is not None:
                response = self._codec.encode_nack(decoded.sequence, fault_code).strip()
            else:
                self._last_valid_frame = time.monotonic()
                response = self._codec.encode_ack(decoded.sequence).strip()
            self._last_sequence = decoded.sequence
            self._last_frame = canonical_frame
            self._last_response = response
            self._next_sequence = decoded.sequence + 1
            return response

    def _apply_command_unlocked(self, command: ControlCommand) -> str | None:
        if self._watchdog_tripped and command.kind not in {
            CommandType.EMERGENCY_STOP,
            CommandType.RESET,
            CommandType.SET_BRAKE,
            CommandType.SET_SPEED,
            CommandType.SET_ARMED,
        }:
            return "WATCHDOG_TIMEOUT"
        if command.kind is CommandType.EMERGENCY_STOP:
            self._enter_safe_state_unlocked()
            self._estopped = True
        elif command.kind is CommandType.RESET:
            self._estopped = False
            self._armed = False
            self._watchdog_tripped = False
        elif command.kind is CommandType.SET_BRAKE:
            requested_brake = bool(command.value)
            if self._watchdog_tripped and not requested_brake:
                return "WATCHDOG_TIMEOUT"
            if not requested_brake and (not self._armed or self._estopped):
                return "ESTOP_LATCHED" if self._estopped else "NOT_ARMED"
            self._braked = requested_brake
            if self._braked:
                self._speed_percent = 0
        elif command.kind is CommandType.SET_SPEED:
            requested_speed = int(command.value) if command.value is not None else 0
            if self._watchdog_tripped and requested_speed != 0:
                return "WATCHDOG_TIMEOUT"
            if requested_speed != 0 and self._estopped:
                return "ESTOP_LATCHED"
            if requested_speed != 0 and (not self._armed or self._braked):
                return "NOT_ARMED"
            self._speed_percent = requested_speed
        elif command.kind is CommandType.SET_STEERING:
            requested_steering = int(command.value) if command.value is not None else 0
            if self._watchdog_tripped and requested_steering != 0:
                return "WATCHDOG_TIMEOUT"
            if requested_steering != 0 and self._estopped:
                return "ESTOP_LATCHED"
            if requested_steering != 0 and not self._armed:
                return "NOT_ARMED"
            self._steering = requested_steering
        elif command.kind is CommandType.SET_ARMED:
            requested_armed = bool(command.value)
            if self._watchdog_tripped and requested_armed:
                return "WATCHDOG_TIMEOUT"
            if requested_armed and self._estopped:
                return "ESTOP_LATCHED"
            self._armed = requested_armed
            if not requested_armed:
                self._enter_safe_state_unlocked()
        return None

    def disconnect(self) -> None:
        with self._lock:
            self._enter_safe_state_unlocked()
            self._connected = False
            self._watchdog_stop.set()
            thread = self._watchdog_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        with self._lock:
            self._watchdog_thread = None

    def _watchdog_loop(self) -> None:
        poll_interval = min(self._watchdog_timeout / 4.0, 0.05)
        while not self._watchdog_stop.wait(poll_interval):
            with self._lock:
                if not self._connected:
                    return
                if time.monotonic() - self._last_valid_frame > self._watchdog_timeout:
                    self._enter_safe_state_unlocked()
                    self._watchdog_tripped = True

    def _enter_safe_state_unlocked(self) -> None:
        self._speed_percent = 0
        self._braked = True
        self._armed = False


class SimulatedLidarSource:
    """Generate a repeatable 180-degree scan with one controllable obstacle."""

    def __init__(self, obstacle_distance_cm: float = 180.0) -> None:
        if not math.isfinite(obstacle_distance_cm) or obstacle_distance_cm <= 0:
            raise ValueError("obstacle distance must be finite and positive")
        self._connected = False
        self._obstacle_distance = obstacle_distance_cm
        self._started = 0.0
        self._lock = threading.Lock()

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def description(self) -> str:
        return "simulation:lidar"

    def connect(self) -> None:
        self._connected = True
        self._started = time.monotonic()

    def set_obstacle_distance(self, distance_cm: float) -> None:
        if not math.isfinite(distance_cm) or distance_cm <= 0:
            raise ValueError("obstacle distance must be finite and positive")
        with self._lock:
            self._obstacle_distance = distance_cm

    def latest_scan(self) -> tuple[tuple[ScanPoint, ...], float] | None:
        if not self._connected:
            return None
        now = time.monotonic()
        with self._lock:
            obstacle = self._obstacle_distance
        phase = (now - self._started) * 0.35
        points: list[ScanPoint] = []
        for angle in range(-90, 91, 3):
            wall_distance = 350.0 + 40.0 * math.sin(math.radians(angle * 2) + phase)
            distance = obstacle if abs(angle) <= 4 else wall_distance
            points.append(ScanPoint(float(angle), float(distance), quality=15))
        return tuple(points), now

    def disconnect(self) -> None:
        self._connected = False


class SimulatedControllerSource:
    """Mutable neutral-by-default controller for automated scenarios."""

    def __init__(self) -> None:
        self._connected = False
        self.snapshot = ControllerSnapshot(connected=False)

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def description(self) -> str:
        return "simulation:controller"

    def connect(self) -> None:
        self._connected = True
        self.snapshot = ControllerSnapshot(connected=True)

    def poll(self) -> ControllerSnapshot:
        if not self._connected:
            return ControllerSnapshot(connected=False)
        return self.snapshot

    def disconnect(self) -> None:
        self._connected = False
        self.snapshot = ControllerSnapshot(connected=False)
