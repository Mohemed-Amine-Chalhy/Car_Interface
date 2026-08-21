"""Thread-safe orchestration of safety decisions, devices, and UI events."""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable
from contextlib import suppress

from car_interface.adapters.base import (
    ControllerSnapshot,
    ControllerSource,
    LidarSource,
    VehicleTransport,
)
from car_interface.adapters.simulated import SimulatedLidarSource
from car_interface.config import AppConfig
from car_interface.domain import (
    ControlCommand,
    DeviceConnection,
    DeviceKind,
    FaultCode,
    ProtocolCodec,
    ProtocolContract,
    SafetyDecision,
    SafetyError,
    SafetyPhase,
    SafetyPolicy,
    SafetyStateMachine,
)

from .dispatcher import CommandDispatcher, DispatchReceipt
from .events import EventBroker, EventType
from .lidar_analysis import LidarAssessment, assess_scan
from .models import ServiceSnapshot

LOGGER = logging.getLogger(__name__)


class ControlService:
    """Coordinate all mutable runtime behavior behind a small asynchronous API."""

    def __init__(
        self,
        config: AppConfig,
        *,
        vehicle: VehicleTransport,
        lidar: LidarSource,
        controller: ControllerSource,
    ) -> None:
        self.config = config.validate()
        self.events = EventBroker()
        self._vehicle = vehicle
        self._lidar = lidar
        self._controller = controller
        policy = SafetyPolicy(
            controller_timeout_seconds=config.command_stale_seconds,
            lidar_timeout_seconds=config.lidar_stale_seconds,
            max_abs_speed_percent=config.max_speed_percent,
            max_abs_steering_percent=100,
        )
        self._machine = SafetyStateMachine(policy=policy)
        contract = ProtocolContract(
            ack_timeout_seconds=config.ack_timeout_seconds,
            heartbeat_interval_seconds=config.heartbeat_interval_seconds,
            watchdog_timeout_seconds=config.command_stale_seconds,
        )
        self._dispatcher = CommandDispatcher(
            vehicle,
            codec=ProtocolCodec(contract),
            require_ack=config.require_ack,
            ack_timeout_seconds=config.ack_timeout_seconds,
            maximum_command_age_seconds=config.command_stale_seconds,
            on_receipt=self._on_receipt,
        )
        self._domain_lock = threading.RLock()
        self._lifecycle_lock = threading.Lock()
        self._monitor_stop = threading.Event()
        self._monitor_thread: threading.Thread | None = None
        self._lifecycle_thread: threading.Thread | None = None
        self._lifecycle_cancel = threading.Event()
        self._desired_speed = 0
        self._desired_steering = 0
        self._direction = "forward"
        self._controller_inputs_neutral = False
        self._resume_requires_neutral = True
        self._auto_stop_enabled = True
        self._closest_obstacle_cm: float | None = None
        self._dispatch_failure_latched = False
        self._shutting_down = False

    def snapshot(self) -> ServiceSnapshot:
        with self._domain_lock:
            state = self._machine.snapshot
            devices = {status.device: status for status in state.devices}
            return ServiceSnapshot(
                phase=state.phase.value,
                mode=state.mode.value,
                vehicle_connected=(
                    devices[DeviceKind.ACTUATOR].connection is DeviceConnection.CONNECTED
                ),
                lidar_connected=(
                    devices[DeviceKind.LIDAR].connection is DeviceConnection.CONNECTED
                ),
                controller_connected=(
                    devices[DeviceKind.CONTROLLER].connection is DeviceConnection.CONNECTED
                ),
                speed_percent=state.speed_percent,
                steering_percent=state.steering_percent,
                direction=self._direction,
                brake_active=state.brake_engaged,
                estop_active=state.emergency_stop_latched,
                auto_stop_enabled=self._auto_stop_enabled,
                closest_obstacle_cm=self._closest_obstacle_cm,
                fault=state.active_fault.detail if state.active_fault else None,
            )

    def connect(self) -> None:
        if self._shutting_down:
            self._publish(EventType.LOG, "The service is shutting down")
            return
        self._start_lifecycle(self._connect_worker, "device-connect", clear_cancel=True)

    def disconnect(self) -> None:
        self._lifecycle_cancel.set()
        self._start_lifecycle(self._disconnect_worker, "device-disconnect")

    def arm(self) -> None:
        with self._domain_lock:
            neutral = self._controller_inputs_neutral
            resume_blocked = self._resume_requires_neutral
            closest_obstacle = self._closest_obstacle_cm
            auto_stop_enabled = self._auto_stop_enabled
        if not neutral or resume_blocked:
            self._publish(
                EventType.LOG,
                "Cannot arm: return controller throttle, steering, and brake to neutral first",
            )
            return
        if (
            auto_stop_enabled
            and closest_obstacle is not None
            and closest_obstacle < self.config.auto_stop_distance_cm
        ):
            self._publish(
                EventType.LOG,
                f"Cannot arm: obstacle at {closest_obstacle:.1f} cm is inside the stop limit",
            )
            return
        self._run_transition("arm", lambda now: self._machine.arm(now))

    def disarm(self) -> None:
        with self._domain_lock:
            self._desired_speed = 0
            self._desired_steering = 0
            self._resume_requires_neutral = True
        self._run_transition("disarm", lambda now: self._machine.disarm(now))

    def set_speed(self, speed_percent: int) -> None:
        if isinstance(speed_percent, bool) or not isinstance(speed_percent, int):
            self._publish(EventType.LOG, "Rejected speed: expected an integer percentage")
            return
        requested = max(0, min(self.config.max_speed_percent, speed_percent))
        with self._domain_lock:
            blocked = False
            if self._resume_requires_neutral:
                if requested == 0 and self._controller_inputs_neutral:
                    self._resume_requires_neutral = False
                elif requested != 0:
                    blocked = True
            if blocked:
                self._publish(
                    EventType.LOG,
                    "Return all controller inputs to neutral and explicitly re-arm before moving.",
                )
                return
            self._desired_speed = requested
            phase = self._machine.snapshot.phase
            desired_steering = self._desired_steering
            direction = self._direction
        if phase not in {SafetyPhase.ARMED, SafetyPhase.DRIVING, SafetyPhase.BRAKING}:
            return
        if phase is SafetyPhase.BRAKING:
            self._publish(EventType.LOG, "Release the brake and explicitly re-arm before moving")
            return
        signed_speed = requested if direction == "forward" else -requested
        self._run_transition(
            "speed",
            lambda now: self._machine.drive(
                signed_speed,
                desired_steering,
                now,
            ),
        )

    def set_steering(self, steering_percent: int) -> None:
        if isinstance(steering_percent, bool) or not isinstance(steering_percent, int):
            self._publish(EventType.LOG, "Rejected steering: expected an integer percentage")
            return
        with self._domain_lock:
            self._desired_steering = max(-100, min(100, steering_percent))
            phase = self._machine.snapshot.phase
            desired_speed = self._desired_speed
            desired_steering = self._desired_steering
            direction = self._direction
        if phase is not SafetyPhase.DRIVING:
            return
        signed_speed = desired_speed if direction == "forward" else -desired_speed
        self._run_transition(
            "steering",
            lambda now: self._machine.drive(
                signed_speed,
                desired_steering,
                now,
            ),
        )

    def set_direction(self, direction: str) -> None:
        normalized = direction.strip().lower()
        if normalized not in {"forward", "reverse"}:
            self._publish(EventType.LOG, f"Rejected unknown direction {direction!r}")
            return
        with self._domain_lock:
            if normalized == self._direction:
                return
            moving = self._machine.snapshot.phase is SafetyPhase.DRIVING
            desired_speed = self._desired_speed
        if moving and desired_speed:
            with self._domain_lock:
                self._desired_speed = 0
                self._desired_steering = 0
                self._resume_requires_neutral = True
            if not self._run_transition(
                "direction-change brake",
                lambda now: self._machine.brake(now, "direction change"),
            ):
                return
            self._publish(
                EventType.LOG,
                "Direction changed only after braking; set speed again to move.",
            )
        with self._domain_lock:
            self._direction = normalized
        self._publish(EventType.STATE, self.snapshot())

    def set_brake(self, active: bool) -> None:
        if active:
            with self._domain_lock:
                self._desired_speed = 0
                self._desired_steering = 0
                self._resume_requires_neutral = True
            self._run_transition("brake", lambda now: self._machine.brake(now, "operator brake"))
            return
        with self._domain_lock:
            phase = self._machine.snapshot.phase
            neutral = self._controller_inputs_neutral
            resume_blocked = self._resume_requires_neutral
        if phase is SafetyPhase.BRAKING:
            if not neutral or resume_blocked:
                self._publish(
                    EventType.LOG,
                    "Cannot release brake: return controller inputs to neutral first",
                )
                return
            self._publish(
                EventType.LOG,
                "Brake remains held; explicitly Arm before the next motion command.",
            )

    def emergency_stop(self, reason: str = "operator") -> None:
        with self._domain_lock:
            self._desired_speed = 0
            self._desired_steering = 0
            self._resume_requires_neutral = True
        self._run_transition(
            "emergency stop",
            lambda now: self._machine.emergency_stop(reason, now),
        )

    def reset_emergency_stop(self) -> None:
        with self._domain_lock:
            self._desired_speed = 0
            self._desired_steering = 0
            self._resume_requires_neutral = True
        self._run_transition("reset", lambda now: self._machine.reset(now))

    def set_auto_stop(self, enabled: bool) -> None:
        with self._domain_lock:
            phase = self._machine.snapshot.phase
            if not enabled and phase in {
                SafetyPhase.ARMED,
                SafetyPhase.DRIVING,
                SafetyPhase.BRAKING,
            }:
                rejected = True
            else:
                rejected = False
                self._auto_stop_enabled = enabled
        if rejected:
            self._publish(
                EventType.LOG,
                "Cannot disable Lidar automatic stop while the vehicle is active",
            )
            self._publish(EventType.STATE, self.snapshot())
            return
        state = "enabled" if enabled else "disabled"
        self._publish(EventType.LOG, f"Lidar automatic stop {state}")
        self._publish(EventType.STATE, self.snapshot())

    def set_simulated_obstacle(self, distance_cm: float) -> None:
        if isinstance(self._lidar, SimulatedLidarSource):
            try:
                self._lidar.set_obstacle_distance(distance_cm)
            except (TypeError, ValueError) as exc:
                self._publish(EventType.LOG, f"Rejected simulated obstacle: {exc}")

    def shutdown(self) -> None:
        self._shutting_down = True
        self._lifecycle_cancel.set()
        self._monitor_stop.set()
        lifecycle = self._lifecycle_thread
        if (
            lifecycle is not None
            and lifecycle.is_alive()
            and lifecycle is not threading.current_thread()
        ):
            lifecycle.join(timeout=12.0)
        self._disconnect_worker()

    def _start_lifecycle(
        self,
        target: Callable[[], None],
        name: str,
        *,
        clear_cancel: bool = False,
    ) -> None:
        with self._lifecycle_lock:
            if self._lifecycle_thread is not None and self._lifecycle_thread.is_alive():
                self._publish(EventType.LOG, "A device lifecycle operation is already running")
                return
            if clear_cancel:
                self._lifecycle_cancel.clear()
            self._lifecycle_thread = threading.Thread(
                target=target,
                name=name,
                daemon=True,
            )
            self._lifecycle_thread.start()

    def _connect_worker(self) -> None:
        if self._connect_cancelled():
            return
        if self.snapshot().vehicle_connected or self._dispatcher.is_running:
            self._publish(EventType.LOG, "Devices are already connected")
            return
        self._publish(EventType.CONNECTION, f"Connecting {self._vehicle.description}")
        self._dispatch_failure_latched = False
        try:
            self._dispatcher.start()
            self._raise_if_connect_cancelled()
            decision = self._transition(lambda now: self._machine.connect(now))
            receipts = self._apply_decision(decision)
            if not self._wait_for_receipts(receipts, 2.0):
                raise RuntimeError("firmware did not acknowledge the initial safe state")
            self._raise_if_connect_cancelled()

            self._controller.connect()
            initial_controller = self._controller.poll()
            if not initial_controller.connected:
                raise RuntimeError("controller disconnected during startup")
            with self._domain_lock:
                self._controller_inputs_neutral = self._is_controller_neutral(initial_controller)
                self._resume_requires_neutral = not self._controller_inputs_neutral
                self._direction = initial_controller.direction
            self._apply_decision(
                self._transition(
                    lambda now: self._machine.device_connected(DeviceKind.CONTROLLER, now)
                )
            )
            self._raise_if_connect_cancelled()
            self._lidar.connect()
            self._wait_for_lidar_ready()
            self._apply_decision(
                self._transition(lambda now: self._machine.device_connected(DeviceKind.LIDAR, now))
            )
            self._raise_if_connect_cancelled()
            self._monitor_stop.clear()
            self._monitor_thread = threading.Thread(
                target=self._monitor_loop,
                name="safety-monitor",
                daemon=True,
            )
            self._monitor_thread.start()
            self._publish(EventType.CONNECTION, "All required devices connected in safe state")
            self._publish(EventType.STATE, self.snapshot())
        except Exception as exc:
            LOGGER.exception("Device connection failed")
            self._publish(EventType.FAULT, f"Connection failed: {exc}")
            self._connection_failed(str(exc))

    def _connection_failed(self, detail: str) -> None:
        with self._domain_lock:
            state = self._machine.snapshot
            if state.phase is not SafetyPhase.DISCONNECTED:
                decision = self._machine.report_fault(
                    FaultCode.COMMUNICATION_ERROR,
                    detail,
                    time.monotonic(),
                )
            else:
                decision = None
        if decision is not None:
            receipts = self._apply_decision(decision)
            self._wait_for_receipts(receipts, 1.0)
        self._monitor_stop.set()
        self._lidar.disconnect()
        self._controller.disconnect()
        self._dispatcher.stop()
        with self._domain_lock:
            self._desired_speed = 0
            self._desired_steering = 0
            self._direction = "forward"
            self._controller_inputs_neutral = False
            self._resume_requires_neutral = True
            self._closest_obstacle_cm = None
            if self._machine.snapshot.phase is not SafetyPhase.DISCONNECTED:
                self._machine.disconnect(time.monotonic())
        self._publish(EventType.STATE, self.snapshot())

    def _disconnect_worker(self) -> None:
        self._monitor_stop.set()
        monitor = self._monitor_thread
        if monitor is not None and monitor.is_alive() and monitor is not threading.current_thread():
            monitor.join(timeout=1.5)
        self._monitor_thread = None

        with self._domain_lock:
            state = self._machine.snapshot
            decision = (
                self._machine.disconnect(time.monotonic())
                if state.phase is not SafetyPhase.DISCONNECTED
                else None
            )
        if decision is not None:
            receipts = self._apply_decision(decision)
            if not self._wait_for_receipts(receipts, 1.5):
                self._publish(
                    EventType.FAULT,
                    "Safe-stop acknowledgement timed out; rely on the firmware watchdog "
                    "and physical E-stop.",
                )
            self._dispatcher.wait_for_idle(0.5)
        self._dispatcher.stop()
        self._lidar.disconnect()
        self._controller.disconnect()
        with self._domain_lock:
            self._desired_speed = 0
            self._desired_steering = 0
            self._direction = "forward"
            self._controller_inputs_neutral = False
            self._resume_requires_neutral = True
            self._closest_obstacle_cm = None
        self._publish(EventType.CONNECTION, "Devices disconnected")
        self._publish(EventType.STATE, self.snapshot())

    def _monitor_loop(self) -> None:
        next_heartbeat = 0.0
        last_controller_connected = True
        last_controller_values: tuple[int, int, bool, str] | None = None
        lidar_stale_reported = False
        last_scan_timestamp: float | None = None
        while not self._monitor_stop.wait(0.03):
            now = time.monotonic()
            try:
                controller = self._controller.poll()
                if not controller.connected:
                    with self._domain_lock:
                        self._controller_inputs_neutral = False
                        self._resume_requires_neutral = True
                    if last_controller_connected:
                        self._apply_decision(
                            self._transition(
                                lambda event_time: self._machine.controller_lost(
                                    event_time,
                                    "controller disconnected while supervision was active",
                                )
                            )
                        )
                        self._publish(EventType.FAULT, "Controller lost: safe stop requested")
                    last_controller_connected = False
                else:
                    last_controller_connected = True
                    controller_neutral = self._is_controller_neutral(controller)
                    with self._domain_lock:
                        self._controller_inputs_neutral = controller_neutral
                        if controller_neutral:
                            self._resume_requires_neutral = False
                    self._transition(
                        lambda event_time: self._machine.record_heartbeat(
                            DeviceKind.CONTROLLER, event_time
                        )
                    )
                    controller_values = (
                        round(controller.throttle * self.config.max_speed_percent),
                        round(controller.steering * 100),
                        controller.brake_pressed,
                        controller.direction,
                    )
                    if last_controller_values is not None:
                        self._handle_controller(controller, last_controller_values)
                    last_controller_values = controller_values

                scan = self._lidar.latest_scan()
                if scan is not None:
                    points, captured_at = scan
                    assessment_now = time.monotonic()
                    assessment = assess_scan(
                        points,
                        captured_at=captured_at,
                        now=assessment_now,
                        vehicle_width_cm=self.config.vehicle_width_cm,
                        stale_after_seconds=self.config.lidar_stale_seconds,
                    )
                    if assessment.stale:
                        if not lidar_stale_reported:
                            self._apply_decision(
                                self._transition(
                                    lambda event_time: self._machine.sensor_lost(
                                        event_time,
                                        "Lidar scan timestamp became stale",
                                    )
                                )
                            )
                            self._publish(
                                EventType.FAULT,
                                "Lidar scan is stale: safe stop requested",
                            )
                            lidar_stale_reported = True
                    else:
                        if (
                            lidar_stale_reported
                            and last_scan_timestamp is not None
                            and captured_at > last_scan_timestamp
                        ):
                            self._apply_decision(
                                self._transition(
                                    lambda event_time: self._machine.device_connected(
                                        DeviceKind.LIDAR, event_time
                                    )
                                )
                            )
                            lidar_stale_reported = False
                        self._transition(
                            lambda event_time: self._machine.record_heartbeat(
                                DeviceKind.LIDAR, event_time
                            )
                        )
                    last_scan_timestamp = captured_at
                    with self._domain_lock:
                        self._closest_obstacle_cm = assessment.closest_in_path_cm
                    self._publish(EventType.SCAN, assessment)
                    self._handle_obstacle(assessment)

                if now >= next_heartbeat and self._dispatcher.is_running:
                    self._dispatcher.submit(ControlCommand.heartbeat())
                    next_heartbeat = now + self.config.heartbeat_interval_seconds

                decision = self._transition(
                    lambda event_time: self._machine.check_staleness(event_time)
                )
                if decision.entered_safe_stop and decision.commands:
                    self._apply_decision(decision)
                    self._publish(EventType.FAULT, decision.reason or "Safety data became stale")
            except Exception as exc:
                LOGGER.exception("Safety monitor iteration failed")
                self._publish(EventType.FAULT, f"Safety monitor error: {exc}")
                self.emergency_stop("safety monitor error")
                return

    def _handle_controller(
        self,
        controller: ControllerSnapshot,
        previous: tuple[int, int, bool, str] | None,
    ) -> None:
        values = (
            round(controller.throttle * self.config.max_speed_percent),
            round(controller.steering * 100),
            controller.brake_pressed,
            controller.direction,
        )
        if values == previous:
            return
        if controller.brake_pressed:
            self.set_brake(True)
            return
        if previous is not None and previous[2]:
            self.set_brake(False)
            return
        self.set_direction(controller.direction)
        with self._domain_lock:
            self._desired_steering = values[1]
        self.set_speed(values[0])

    def _handle_obstacle(self, assessment: LidarAssessment) -> None:
        closest = assessment.closest_in_path_cm
        with self._domain_lock:
            if not self._auto_stop_enabled or closest is None:
                return
            if self._machine.snapshot.emergency_stop_latched:
                return
            active = self._machine.snapshot.phase in {
                SafetyPhase.ARMED,
                SafetyPhase.DRIVING,
                SafetyPhase.BRAKING,
            }
        if active and closest < self.config.auto_stop_distance_cm:
            self.emergency_stop(f"Lidar obstacle at {closest:.1f} cm inside projected path")

    def _connect_cancelled(self) -> bool:
        return self._shutting_down or self._lifecycle_cancel.is_set()

    def _wait_for_lidar_ready(self) -> None:
        deadline = time.monotonic() + max(2.0, self.config.lidar_stale_seconds * 2.0)
        while time.monotonic() < deadline:
            self._raise_if_connect_cancelled()
            scan = self._lidar.latest_scan()
            now = time.monotonic()
            if scan is not None:
                points, captured_at = scan
                if (
                    points
                    and captured_at <= now
                    and now - captured_at <= self.config.lidar_stale_seconds
                ):
                    return
            time.sleep(0.02)
        raise RuntimeError("Lidar did not provide a fresh, non-empty startup scan")

    def _raise_if_connect_cancelled(self) -> None:
        if self._connect_cancelled():
            raise RuntimeError("device connection cancelled")

    @staticmethod
    def _is_controller_neutral(controller: ControllerSnapshot) -> bool:
        return (
            controller.connected
            and abs(controller.throttle) <= 0.02
            and abs(controller.steering) <= 0.05
            and not controller.brake_pressed
        )

    def _run_transition(
        self,
        label: str,
        operation: Callable[[float], SafetyDecision],
    ) -> bool:
        try:
            decision = self._transition(operation)
            self._apply_decision(decision)
            return True
        except (SafetyError, ValueError, TypeError) as exc:
            self._publish(EventType.LOG, f"Cannot {label}: {exc}")
            return False

    def _transition(
        self,
        operation: Callable[[float], SafetyDecision],
    ) -> SafetyDecision:
        with self._domain_lock:
            return operation(time.monotonic())

    def _apply_decision(self, decision: SafetyDecision) -> tuple[DispatchReceipt, ...]:
        self._publish(EventType.STATE, self.snapshot())
        if not decision.commands:
            return ()
        try:
            receipts = self._dispatcher.submit_decision(decision)
        except BufferError as exc:
            self._publish(EventType.FAULT, f"Command queue failure: {exc}")
            self.emergency_stop("command queue saturation")
            return ()
        self._publish(
            EventType.LOG,
            f"{decision.action}: {decision.reason or 'state updated'}",
        )
        return receipts

    @staticmethod
    def _wait_for_receipts(receipts: tuple[DispatchReceipt, ...], timeout_seconds: float) -> bool:
        deadline = time.monotonic() + timeout_seconds
        for receipt in receipts:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not receipt.wait(remaining):
                return False
        return True

    def _on_receipt(self, receipt: DispatchReceipt) -> None:
        if receipt.cancelled:
            return
        if receipt.succeeded:
            self._publish(
                EventType.COMMAND,
                f"ACK seq={receipt.sequence} {receipt.command.kind.value}",
            )
            if receipt.command.kind.value == "HBT":
                with suppress(SafetyError, ValueError):
                    self._transition(
                        lambda now: self._machine.record_heartbeat(DeviceKind.ACTUATOR, now)
                    )
            return
        self._publish(
            EventType.FAULT,
            f"Command {receipt.command.kind.value} failed: {receipt.error}",
        )
        if self._dispatch_failure_latched or self._shutting_down:
            return
        self._dispatch_failure_latched = True
        with self._domain_lock:
            self._desired_speed = 0
            self._desired_steering = 0
            self._resume_requires_neutral = True
            if self._machine.snapshot.phase is not SafetyPhase.DISCONNECTED:
                self._machine.device_lost(
                    DeviceKind.ACTUATOR,
                    time.monotonic(),
                    receipt.error or "actuator command failed",
                )
        self._monitor_stop.set()
        self._dispatcher.stop()
        self._publish(EventType.STATE, self.snapshot())

    def _publish(self, event_type: EventType, payload: object) -> None:
        self.events.publish(event_type, payload)
        if event_type is EventType.FAULT:
            LOGGER.error("%s", payload)
        elif event_type in {EventType.LOG, EventType.CONNECTION}:
            LOGGER.info("%s", payload)
