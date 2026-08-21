"""Deterministic vehicle safety state machine."""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import isfinite
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Iterable

from .commands import ControlCommand
from .state import (
    ControlMode,
    ControlState,
    DeviceConnection,
    DeviceKind,
    DeviceStatus,
    Fault,
    FaultCode,
    SafetyPhase,
)


class SafetyError(RuntimeError):
    """Base class for rejected safety operations."""


class InvalidTransition(SafetyError):
    """Raised when an action is not valid in the current phase."""

    def __init__(self, action: str, current: SafetyPhase, allowed: Iterable[SafetyPhase]) -> None:
        self.action = action
        self.current = current
        self.allowed = tuple(allowed)
        expected = ", ".join(phase.value for phase in self.allowed)
        super().__init__(f"cannot {action} while {current.value}; expected one of: {expected}")


class SafetyPrerequisiteError(SafetyError):
    """Raised when required devices or freshness checks prevent an action."""


@dataclass(frozen=True, slots=True)
class SafetyPolicy:
    """Limits and freshness requirements applied by the state machine."""

    required_devices: tuple[DeviceKind, ...] = (
        DeviceKind.ACTUATOR,
        DeviceKind.CONTROLLER,
        DeviceKind.LIDAR,
    )
    controller_timeout_seconds: float = 0.75
    lidar_timeout_seconds: float = 0.5
    max_abs_speed_percent: int = 100
    max_abs_steering_percent: int = 100

    def __post_init__(self) -> None:
        if len(self.required_devices) != len(set(self.required_devices)):
            raise ValueError("required_devices must not contain duplicates")
        if DeviceKind.ACTUATOR not in self.required_devices:
            raise ValueError("the actuator must always be a required device")
        for field_name, value in (
            ("controller_timeout_seconds", self.controller_timeout_seconds),
            ("lidar_timeout_seconds", self.lidar_timeout_seconds),
        ):
            if not isfinite(value) or value <= 0:
                raise ValueError(f"{field_name} must be finite and positive")
        for field_name, value in (
            ("max_abs_speed_percent", self.max_abs_speed_percent),
            ("max_abs_steering_percent", self.max_abs_steering_percent),
        ):
            if isinstance(value, bool) or not isinstance(value, int):
                raise TypeError(f"{field_name} must be an integer")
            if not 1 <= value <= 100:
                raise ValueError(f"{field_name} must be between 1 and 100")

    def timeout_for(self, device: DeviceKind) -> float | None:
        """Return the freshness timeout, or ``None`` for connection-only health."""

        return {
            DeviceKind.ACTUATOR: None,
            DeviceKind.CONTROLLER: self.controller_timeout_seconds,
            DeviceKind.LIDAR: self.lidar_timeout_seconds,
        }[device]


@dataclass(frozen=True, slots=True)
class SafetyDecision:
    """A state transition and the commands required to enact it."""

    action: str
    previous: ControlState
    current: ControlState
    commands: tuple[ControlCommand, ...] = ()
    discard_pending_commands: bool = False
    reason: str | None = None

    @property
    def changed(self) -> bool:
        return self.previous != self.current

    @property
    def entered_safe_stop(self) -> bool:
        return (
            self.current.phase
            in {
                SafetyPhase.BRAKING,
                SafetyPhase.EMERGENCY_STOP,
                SafetyPhase.FAULT,
                SafetyPhase.DISCONNECTED,
            }
            and self.current.brake_engaged
        )


def resolve_command_queue(
    pending: Iterable[ControlCommand], decision: SafetyDecision
) -> tuple[ControlCommand, ...]:
    """Merge a decision into a queue while enforcing safety priority.

    Safety decisions that set ``discard_pending_commands`` atomically evict the
    old queue.  Commands are then stable-sorted by priority, guaranteeing that
    an emergency-stop opcode precedes zero-speed/brake commands and that no old
    motion command can run after it.
    """

    commands = (
        list(decision.commands)
        if decision.discard_pending_commands
        else [*pending, *decision.commands]
    )
    return tuple(sorted(commands, key=lambda command: command.priority))


class SafetyStateMachine:
    """Pure event-driven state machine for vehicle safety.

    The caller owns time and I/O.  Every public event accepts a monotonic
    timestamp and returns a :class:`SafetyDecision`; the returned commands are
    intentions only and are never sent by this class.
    """

    def __init__(
        self,
        policy: SafetyPolicy | None = None,
        *,
        initial_mode: ControlMode = ControlMode.MANUAL,
    ) -> None:
        self.policy = policy or SafetyPolicy()
        devices = tuple(
            DeviceStatus(
                device=device,
                required=device in self.policy.required_devices,
            )
            for device in DeviceKind
        )
        self._state = ControlState(
            phase=SafetyPhase.DISCONNECTED,
            mode=initial_mode,
            speed_percent=0,
            steering_percent=0,
            brake_engaged=True,
            emergency_stop_latched=False,
            devices=devices,
        )

    @property
    def snapshot(self) -> ControlState:
        return self._state

    def connect(self, now: float) -> SafetyDecision:
        """Record an established actuator link and force a neutral safe state."""

        now = self._validate_event_time(now)
        self._require_phase("connect", SafetyPhase.DISCONNECTED)
        previous = self._state
        self._replace_device(
            DeviceKind.ACTUATOR,
            connection=DeviceConnection.CONNECTED,
            last_seen_seconds=now,
            detail=None,
        )
        target = SafetyPhase.SAFE_CONNECTED
        if self._state.emergency_stop_latched:
            target = SafetyPhase.EMERGENCY_STOP
        elif self._state.active_fault is not None:
            target = SafetyPhase.FAULT
        return self._commit(
            action="connect",
            previous=previous,
            now=now,
            phase=target,
            speed_percent=0,
            steering_percent=0,
            brake_engaged=True,
            commands=self._neutral_commands(),
            discard_pending=True,
            reason="actuator connected in a safe, neutral state",
        )

    def device_connected(self, device: DeviceKind, now: float) -> SafetyDecision:
        """Record successful connection or recovery of a supporting device."""

        now = self._validate_event_time(now)
        previous = self._state
        self._replace_device(
            device,
            connection=DeviceConnection.CONNECTED,
            last_seen_seconds=now,
            detail=None,
        )
        return self._commit(
            action="device_connected",
            previous=previous,
            now=now,
            reason=f"{device.value} connected",
        )

    def record_heartbeat(self, device: DeviceKind, now: float) -> SafetyDecision:
        """Mark a device as connected and refresh its monotonic heartbeat."""

        now = self._validate_event_time(now)
        previous = self._state
        self._replace_device(
            device,
            connection=DeviceConnection.CONNECTED,
            last_seen_seconds=now,
            detail=None,
        )
        return self._commit(
            action="record_heartbeat",
            previous=previous,
            now=now,
            reason=f"heartbeat from {device.value}",
        )

    def set_mode(self, mode: ControlMode, now: float) -> SafetyDecision:
        now = self._validate_event_time(now)
        self._require_phase(
            "set mode", SafetyPhase.SAFE_CONNECTED, SafetyPhase.ARMED, SafetyPhase.BRAKING
        )
        previous = self._state
        return self._commit(
            action="set_mode",
            previous=previous,
            now=now,
            mode=mode,
            commands=(ControlCommand.set_mode(mode),),
            reason=f"control mode changed to {mode.value}",
        )

    def arm(self, now: float) -> SafetyDecision:
        """Arm motion only when every required device is connected and fresh."""

        now = self._validate_event_time(now)
        self._require_phase("arm", SafetyPhase.SAFE_CONNECTED, SafetyPhase.BRAKING)
        self._require_healthy_devices(now)
        previous = self._state
        return self._commit(
            action="arm",
            previous=previous,
            now=now,
            phase=SafetyPhase.ARMED,
            speed_percent=0,
            steering_percent=0,
            brake_engaged=True,
            commands=(
                ControlCommand.set_speed(0),
                ControlCommand.set_steering(0),
                ControlCommand.set_brake(True),
                ControlCommand.set_armed(True),
            ),
            reason="all required devices are healthy",
        )

    def drive(self, speed_percent: int, steering_percent: int, now: float) -> SafetyDecision:
        """Apply a bounded motion request or enter fault if health went stale."""

        now = self._validate_event_time(now)
        self._require_phase("drive", SafetyPhase.ARMED, SafetyPhase.DRIVING)
        self._validate_motion(speed_percent, steering_percent)
        unhealthy = self._first_unhealthy_device(now)
        if unhealthy is not None:
            status, stale = unhealthy
            detail = (
                f"{status.device.value} data exceeded its freshness timeout"
                if stale
                else f"required device {status.device.value} is not connected"
            )
            code = FaultCode.STALE_DATA if stale else self._loss_code(status.device)
            return self.report_fault(
                code=code,
                detail=detail,
                now=now,
                source=status.device,
            )

        previous = self._state
        commands: tuple[ControlCommand, ...]
        if previous.phase is SafetyPhase.ARMED:
            commands = (
                ControlCommand.set_steering(steering_percent),
                ControlCommand.set_brake(False),
                ControlCommand.set_speed(speed_percent),
            )
        else:
            commands = (
                ControlCommand.set_steering(steering_percent),
                ControlCommand.set_speed(speed_percent),
            )
        return self._commit(
            action="drive",
            previous=previous,
            now=now,
            phase=SafetyPhase.DRIVING,
            speed_percent=speed_percent,
            steering_percent=steering_percent,
            brake_engaged=False,
            commands=commands,
            reason="validated motion request",
        )

    def brake(self, now: float, reason: str = "operator brake") -> SafetyDecision:
        """Safely stop from any phase; this operation never rejects for phase."""

        now = self._validate_event_time(now)
        previous = self._state
        phase = previous.phase
        if phase in {SafetyPhase.ARMED, SafetyPhase.DRIVING, SafetyPhase.BRAKING}:
            phase = SafetyPhase.BRAKING
        commands = () if not self._actuator_connected() else self._stop_commands()
        return self._commit(
            action="brake",
            previous=previous,
            now=now,
            phase=phase,
            speed_percent=0,
            steering_percent=0,
            brake_engaged=True,
            commands=commands,
            discard_pending=True,
            reason=reason,
        )

    def disarm(self, now: float) -> SafetyDecision:
        now = self._validate_event_time(now)
        self._require_phase("disarm", SafetyPhase.ARMED, SafetyPhase.DRIVING, SafetyPhase.BRAKING)
        previous = self._state
        return self._commit(
            action="disarm",
            previous=previous,
            now=now,
            phase=SafetyPhase.SAFE_CONNECTED,
            speed_percent=0,
            steering_percent=0,
            brake_engaged=True,
            commands=self._neutral_commands(),
            discard_pending=True,
            reason="vehicle disarmed",
        )

    def emergency_stop(self, reason: str, now: float) -> SafetyDecision:
        """Latch E-stop and put its opcode ahead of all other commands."""

        if not reason.strip():
            raise ValueError("emergency-stop reason must not be empty")
        now = self._validate_event_time(now)
        previous = self._state
        phase = (
            SafetyPhase.EMERGENCY_STOP if self._actuator_connected() else SafetyPhase.DISCONNECTED
        )
        commands = () if not self._actuator_connected() else self._emergency_commands()
        return self._commit(
            action="emergency_stop",
            previous=previous,
            now=now,
            phase=phase,
            speed_percent=0,
            steering_percent=0,
            brake_engaged=True,
            emergency_stop_latched=True,
            commands=commands,
            discard_pending=True,
            reason=reason,
        )

    def report_fault(
        self,
        code: FaultCode,
        detail: str,
        now: float,
        source: DeviceKind | None = None,
    ) -> SafetyDecision:
        """Latch a fault and request an emergency safe stop when connected."""

        now = self._validate_event_time(now)
        fault = Fault(code=code, detail=detail, occurred_at_seconds=now, source=source)
        previous = self._state
        connected = self._actuator_connected()
        phase = SafetyPhase.FAULT if connected else SafetyPhase.DISCONNECTED
        if previous.emergency_stop_latched and connected:
            phase = SafetyPhase.EMERGENCY_STOP
        commands = self._emergency_commands() if connected else ()
        return self._commit(
            action="report_fault",
            previous=previous,
            now=now,
            phase=phase,
            speed_percent=0,
            steering_percent=0,
            brake_engaged=True,
            active_fault=fault,
            replace_active_fault=True,
            commands=commands,
            discard_pending=True,
            reason=detail,
        )

    def device_lost(
        self, device: DeviceKind, now: float, detail: str | None = None
    ) -> SafetyDecision:
        """Record loss and fail safe when the device is required."""

        now = self._validate_event_time(now)
        message = detail or f"{device.value} connection lost"
        if not message.strip():
            raise ValueError("device-loss detail must not be empty")
        previous = self._state
        self._replace_device(
            device,
            connection=DeviceConnection.DISCONNECTED,
            last_seen_seconds=self._state.device_status(device).last_seen_seconds,
            detail=message,
        )

        if device is DeviceKind.ACTUATOR:
            fault = Fault(
                code=FaultCode.ACTUATOR_LOST,
                detail=message,
                occurred_at_seconds=now,
                source=device,
            )
            return self._commit(
                action="device_lost",
                previous=previous,
                now=now,
                phase=SafetyPhase.DISCONNECTED,
                speed_percent=0,
                steering_percent=0,
                brake_engaged=True,
                active_fault=fault,
                replace_active_fault=True,
                commands=(),
                discard_pending=True,
                reason=message,
            )

        status = self._state.device_status(device)
        if status.required and previous.phase is not SafetyPhase.DISCONNECTED:
            return replace(
                self.report_fault(
                    code=self._loss_code(device),
                    detail=message,
                    now=now,
                    source=device,
                ),
                action="device_lost",
                previous=previous,
            )

        return self._commit(
            action="device_lost",
            previous=previous,
            now=now,
            reason=message,
        )

    def controller_lost(
        self, now: float, detail: str = "controller connection lost"
    ) -> SafetyDecision:
        return self.device_lost(DeviceKind.CONTROLLER, now, detail)

    def sensor_lost(self, now: float, detail: str = "Lidar connection lost") -> SafetyDecision:
        return self.device_lost(DeviceKind.LIDAR, now, detail)

    def check_staleness(self, now: float) -> SafetyDecision:
        """Mark timed-out devices stale and fail safe on the first required one."""

        now = self._validate_event_time(now)
        previous = self._state
        stale_statuses: list[DeviceStatus] = []
        for status in self._state.devices:
            timeout = self.policy.timeout_for(status.device)
            if (
                timeout is not None
                and status.connection is DeviceConnection.CONNECTED
                and not status.is_fresh(now=now, timeout_seconds=timeout)
            ):
                self._replace_device(
                    status.device,
                    connection=DeviceConnection.STALE,
                    last_seen_seconds=status.last_seen_seconds,
                    detail=f"no update within {timeout:g} seconds",
                )
                stale_statuses.append(self._state.device_status(status.device))

        required_stale = next((status for status in stale_statuses if status.required), None)
        if required_stale is not None and previous.phase is not SafetyPhase.DISCONNECTED:
            detail = f"{required_stale.device.value} data is stale"
            return replace(
                self.report_fault(
                    code=FaultCode.STALE_DATA,
                    detail=detail,
                    now=now,
                    source=required_stale.device,
                ),
                action="check_staleness",
                previous=previous,
            )

        return self._commit(
            action="check_staleness",
            previous=previous,
            now=now,
            reason=(
                "stale devices: " + ", ".join(status.device.value for status in stale_statuses)
                if stale_statuses
                else "all connected device data is fresh"
            ),
        )

    def reset(self, now: float) -> SafetyDecision:
        """Explicitly clear a latched E-stop/fault after health is restored."""

        now = self._validate_event_time(now)
        self._require_phase("reset", SafetyPhase.EMERGENCY_STOP, SafetyPhase.FAULT)
        self._require_healthy_devices(now)
        previous = self._state
        return self._commit(
            action="reset",
            previous=previous,
            now=now,
            phase=SafetyPhase.SAFE_CONNECTED,
            speed_percent=0,
            steering_percent=0,
            brake_engaged=True,
            emergency_stop_latched=False,
            active_fault=None,
            replace_active_fault=True,
            commands=(ControlCommand.reset(), *self._neutral_commands()),
            discard_pending=True,
            reason="operator reset after health validation",
        )

    def disconnect(self, now: float) -> SafetyDecision:
        """Request a best-effort safe stop, then mark every device disconnected."""

        now = self._validate_event_time(now)
        previous = self._state
        commands = self._neutral_commands() if self._actuator_connected() else ()
        for device in DeviceKind:
            old = self._state.device_status(device)
            self._replace_device(
                device,
                connection=DeviceConnection.DISCONNECTED,
                last_seen_seconds=old.last_seen_seconds,
                detail="application disconnected",
            )
        return self._commit(
            action="disconnect",
            previous=previous,
            now=now,
            phase=SafetyPhase.DISCONNECTED,
            speed_percent=0,
            steering_percent=0,
            brake_engaged=True,
            commands=commands,
            discard_pending=True,
            reason="application disconnected",
        )

    def _commit(
        self,
        *,
        action: str,
        previous: ControlState,
        now: float,
        commands: tuple[ControlCommand, ...] = (),
        discard_pending: bool = False,
        reason: str | None = None,
        phase: SafetyPhase | None = None,
        mode: ControlMode | None = None,
        speed_percent: int | None = None,
        steering_percent: int | None = None,
        brake_engaged: bool | None = None,
        emergency_stop_latched: bool | None = None,
        active_fault: Fault | None = None,
        replace_active_fault: bool = False,
    ) -> SafetyDecision:
        current = self._state
        self._state = ControlState(
            phase=current.phase if phase is None else phase,
            mode=current.mode if mode is None else mode,
            speed_percent=(current.speed_percent if speed_percent is None else speed_percent),
            steering_percent=(
                current.steering_percent if steering_percent is None else steering_percent
            ),
            brake_engaged=(current.brake_engaged if brake_engaged is None else brake_engaged),
            emergency_stop_latched=(
                current.emergency_stop_latched
                if emergency_stop_latched is None
                else emergency_stop_latched
            ),
            devices=current.devices,
            active_fault=(active_fault if replace_active_fault else current.active_fault),
            revision=previous.revision + 1,
            last_event_seconds=now,
        )
        return SafetyDecision(
            action=action,
            previous=previous,
            current=self._state,
            commands=commands,
            discard_pending_commands=discard_pending,
            reason=reason,
        )

    def _replace_device(
        self,
        device: DeviceKind,
        *,
        connection: DeviceConnection,
        last_seen_seconds: float | None,
        detail: str | None,
    ) -> None:
        old = self._state.device_status(device)
        replacement = replace(
            old,
            connection=connection,
            last_seen_seconds=last_seen_seconds,
            detail=detail,
        )
        devices = tuple(
            replacement if status.device is device else status for status in self._state.devices
        )
        self._state = replace(self._state, devices=devices)

    def _require_phase(self, action: str, *allowed: SafetyPhase) -> None:
        if self._state.phase not in allowed:
            raise InvalidTransition(action, self._state.phase, allowed)

    def _require_healthy_devices(self, now: float) -> None:
        unhealthy = self._first_unhealthy_device(now)
        if unhealthy is None:
            return
        status, stale = unhealthy
        problem = "stale" if stale else status.connection.value
        raise SafetyPrerequisiteError(f"required device {status.device.value} is {problem}")

    def _first_unhealthy_device(self, now: float) -> tuple[DeviceStatus, bool] | None:
        for device in self.policy.required_devices:
            status = self._state.device_status(device)
            timeout = self.policy.timeout_for(device)
            if status.connection is not DeviceConnection.CONNECTED:
                return status, status.connection is DeviceConnection.STALE
            if not status.is_fresh(now=now, timeout_seconds=timeout):
                return status, True
        return None

    def _validate_motion(self, speed_percent: int, steering_percent: int) -> None:
        # Construct first so the shared wire-contract limits are enforced too.
        ControlCommand.set_speed(speed_percent)
        ControlCommand.set_steering(steering_percent)
        if abs(speed_percent) > self.policy.max_abs_speed_percent:
            raise ValueError(
                f"speed exceeds configured limit of {self.policy.max_abs_speed_percent}%"
            )
        if abs(steering_percent) > self.policy.max_abs_steering_percent:
            raise ValueError(
                f"steering exceeds configured limit of {self.policy.max_abs_steering_percent}%"
            )

    def _validate_event_time(self, now: float) -> float:
        if isinstance(now, bool) or not isinstance(now, int | float):
            raise TypeError("now must be a number of monotonic seconds")
        value = float(now)
        if not isfinite(value) or value < 0:
            raise ValueError("now must be finite and non-negative")
        previous = self._state.last_event_seconds
        if previous is not None and value < previous:
            raise ValueError("event timestamps must be monotonic")
        return value

    def _actuator_connected(self) -> bool:
        return (
            self._state.device_status(DeviceKind.ACTUATOR).connection is DeviceConnection.CONNECTED
        )

    @staticmethod
    def _loss_code(device: DeviceKind) -> FaultCode:
        if device is DeviceKind.ACTUATOR:
            return FaultCode.ACTUATOR_LOST
        if device is DeviceKind.CONTROLLER:
            return FaultCode.CONTROLLER_LOST
        return FaultCode.SENSOR_LOST

    @staticmethod
    def _stop_commands() -> tuple[ControlCommand, ...]:
        return (ControlCommand.set_speed(0), ControlCommand.set_brake(True))

    @staticmethod
    def _neutral_commands() -> tuple[ControlCommand, ...]:
        return (
            ControlCommand.set_speed(0),
            ControlCommand.set_steering(0),
            ControlCommand.set_brake(True),
            ControlCommand.set_armed(False),
        )

    @staticmethod
    def _emergency_commands() -> tuple[ControlCommand, ...]:
        return (
            ControlCommand.emergency_stop(),
            ControlCommand.set_speed(0),
            ControlCommand.set_brake(True),
            ControlCommand.set_armed(False),
        )
