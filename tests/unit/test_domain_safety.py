from __future__ import annotations

import unittest

from car_interface.domain import (
    CommandType,
    ControlCommand,
    ControlMode,
    DeviceConnection,
    DeviceKind,
    FaultCode,
    InvalidTransition,
    SafetyPhase,
    SafetyPolicy,
    SafetyPrerequisiteError,
    SafetyStateMachine,
    resolve_command_queue,
)


def connected_machine(*, policy: SafetyPolicy | None = None) -> SafetyStateMachine:
    machine = SafetyStateMachine(policy)
    machine.connect(0.0)
    machine.device_connected(DeviceKind.CONTROLLER, 0.01)
    machine.device_connected(DeviceKind.LIDAR, 0.02)
    return machine


def armed_machine(*, policy: SafetyPolicy | None = None) -> SafetyStateMachine:
    machine = connected_machine(policy=policy)
    machine.arm(0.03)
    return machine


class SafetyTransitionTests(unittest.TestCase):
    def test_happy_path_connect_arm_drive_brake_disarm(self) -> None:
        machine = connected_machine()
        self.assertEqual(machine.snapshot.phase, SafetyPhase.SAFE_CONNECTED)
        self.assertTrue(machine.snapshot.brake_engaged)

        arm = machine.arm(0.03)
        self.assertEqual(arm.current.phase, SafetyPhase.ARMED)
        self.assertTrue(arm.current.brake_engaged)

        drive = machine.drive(35, -20, 0.04)
        self.assertEqual(drive.current.phase, SafetyPhase.DRIVING)
        self.assertEqual(drive.current.speed_percent, 35)
        self.assertEqual(drive.current.steering_percent, -20)
        self.assertFalse(drive.current.brake_engaged)

        brake = machine.brake(0.05)
        self.assertEqual(brake.current.phase, SafetyPhase.BRAKING)
        self.assertEqual(brake.current.speed_percent, 0)
        self.assertTrue(brake.current.brake_engaged)
        self.assertTrue(brake.discard_pending_commands)

        disarm = machine.disarm(0.06)
        self.assertEqual(disarm.current.phase, SafetyPhase.SAFE_CONNECTED)

    def test_illegal_transitions_are_explicit(self) -> None:
        machine = SafetyStateMachine()
        with self.assertRaises(InvalidTransition):
            machine.drive(1, 0, 0.0)
        with self.assertRaises(InvalidTransition):
            machine.arm(0.0)

        machine.connect(0.0)
        with self.assertRaises(InvalidTransition):
            machine.connect(0.01)
        with self.assertRaises(InvalidTransition):
            machine.reset(0.02)

    def test_arm_requires_every_required_device(self) -> None:
        machine = SafetyStateMachine()
        machine.connect(0.0)
        with self.assertRaisesRegex(SafetyPrerequisiteError, "controller"):
            machine.arm(0.1)
        machine.device_connected(DeviceKind.CONTROLLER, 0.2)
        with self.assertRaisesRegex(SafetyPrerequisiteError, "lidar"):
            machine.arm(0.3)

    def test_mode_cannot_change_while_driving(self) -> None:
        machine = armed_machine()
        machine.drive(20, 0, 0.04)
        with self.assertRaises(InvalidTransition):
            machine.set_mode(ControlMode.AUTONOMOUS, 0.05)

    def test_policy_motion_limits_are_enforced(self) -> None:
        machine = armed_machine(
            policy=SafetyPolicy(
                max_abs_speed_percent=40,
                max_abs_steering_percent=30,
            )
        )
        with self.assertRaisesRegex(ValueError, "speed exceeds"):
            machine.drive(41, 0, 0.04)
        with self.assertRaisesRegex(ValueError, "steering exceeds"):
            machine.drive(20, -31, 0.05)

    def test_event_timestamps_must_be_monotonic(self) -> None:
        machine = SafetyStateMachine()
        machine.connect(2.0)
        with self.assertRaisesRegex(ValueError, "monotonic"):
            machine.device_connected(DeviceKind.CONTROLLER, 1.0)


class SafetyStopTests(unittest.TestCase):
    def test_emergency_stop_evicts_motion_and_is_first(self) -> None:
        machine = armed_machine()
        machine.drive(60, 15, 0.04)
        decision = machine.emergency_stop("red button", 0.05)

        queued = resolve_command_queue(
            (
                ControlCommand.set_speed(90),
                ControlCommand.set_steering(50),
                ControlCommand.heartbeat(),
            ),
            decision,
        )
        self.assertEqual(queued[0].kind, CommandType.EMERGENCY_STOP)
        self.assertNotIn(ControlCommand.set_speed(90), queued)
        self.assertNotIn(ControlCommand.set_steering(50), queued)
        self.assertEqual(decision.current.phase, SafetyPhase.EMERGENCY_STOP)
        self.assertEqual(decision.current.speed_percent, 0)
        self.assertTrue(decision.current.brake_engaged)
        self.assertTrue(decision.current.emergency_stop_latched)
        with self.assertRaises(InvalidTransition):
            machine.drive(1, 0, 0.06)

    def test_disconnected_emergency_stop_is_latched_across_connect(self) -> None:
        machine = SafetyStateMachine()
        decision = machine.emergency_stop("offline stop", 0.0)
        self.assertEqual(decision.current.phase, SafetyPhase.DISCONNECTED)
        self.assertTrue(decision.current.emergency_stop_latched)
        self.assertEqual(decision.commands, ())

        reconnected = machine.connect(0.1)
        self.assertEqual(reconnected.current.phase, SafetyPhase.EMERGENCY_STOP)
        self.assertTrue(reconnected.current.brake_engaged)

    def test_reset_requires_restored_health_then_returns_safe_connected(self) -> None:
        machine = armed_machine()
        machine.controller_lost(0.04)
        with self.assertRaises(SafetyPrerequisiteError):
            machine.reset(0.05)

        machine.device_connected(DeviceKind.CONTROLLER, 0.06)
        machine.record_heartbeat(DeviceKind.LIDAR, 0.06)
        reset = machine.reset(0.07)
        self.assertEqual(reset.current.phase, SafetyPhase.SAFE_CONNECTED)
        self.assertIsNone(reset.current.active_fault)
        self.assertFalse(reset.current.emergency_stop_latched)

    def test_intentional_disconnect_requests_safe_commands_and_clears_queue(self) -> None:
        machine = armed_machine()
        machine.drive(-20, 10, 0.04)
        decision = machine.disconnect(0.05)
        self.assertEqual(decision.current.phase, SafetyPhase.DISCONNECTED)
        self.assertEqual(decision.current.speed_percent, 0)
        self.assertTrue(decision.discard_pending_commands)
        self.assertIn(ControlCommand.set_speed(0), decision.commands)
        self.assertTrue(
            all(
                status.connection is DeviceConnection.DISCONNECTED
                for status in decision.current.devices
            )
        )


class DeviceFailureTests(unittest.TestCase):
    def test_controller_loss_during_motion_enters_fault_and_safe_stops(self) -> None:
        machine = armed_machine()
        machine.drive(70, 0, 0.04)
        decision = machine.controller_lost(0.05)
        self.assertEqual(decision.current.phase, SafetyPhase.FAULT)
        self.assertEqual(decision.current.active_fault.code, FaultCode.CONTROLLER_LOST)  # type: ignore[union-attr]
        self.assertEqual(decision.current.speed_percent, 0)
        self.assertTrue(decision.current.brake_engaged)
        self.assertTrue(decision.discard_pending_commands)
        self.assertEqual(decision.commands[0].kind, CommandType.EMERGENCY_STOP)

    def test_required_sensor_loss_enters_fault(self) -> None:
        machine = armed_machine()
        decision = machine.sensor_lost(0.04)
        self.assertEqual(decision.current.phase, SafetyPhase.FAULT)
        self.assertEqual(decision.current.active_fault.code, FaultCode.SENSOR_LOST)  # type: ignore[union-attr]
        self.assertEqual(decision.current.active_fault.source, DeviceKind.LIDAR)  # type: ignore[union-attr]

    def test_stale_required_data_is_detected_and_latched(self) -> None:
        machine = armed_machine()
        decision = machine.check_staleness(0.60)
        self.assertEqual(decision.current.phase, SafetyPhase.FAULT)
        self.assertEqual(decision.current.active_fault.code, FaultCode.STALE_DATA)  # type: ignore[union-attr]
        self.assertEqual(decision.current.active_fault.source, DeviceKind.LIDAR)  # type: ignore[union-attr]
        self.assertEqual(
            decision.current.device_status(DeviceKind.LIDAR).connection,
            DeviceConnection.STALE,
        )

    def test_drive_event_itself_fails_safe_when_data_went_stale(self) -> None:
        machine = armed_machine()
        decision = machine.drive(20, 0, 1.0)
        self.assertEqual(decision.current.phase, SafetyPhase.FAULT)
        self.assertEqual(decision.current.speed_percent, 0)
        self.assertTrue(decision.discard_pending_commands)

    def test_actuator_loss_records_fault_and_reconnects_into_fault_state(self) -> None:
        machine = armed_machine()
        lost = machine.device_lost(DeviceKind.ACTUATOR, 0.04)
        self.assertEqual(lost.current.phase, SafetyPhase.DISCONNECTED)
        self.assertEqual(lost.commands, ())
        self.assertEqual(lost.current.active_fault.code, FaultCode.ACTUATOR_LOST)  # type: ignore[union-attr]

        reconnected = machine.connect(0.05)
        self.assertEqual(reconnected.current.phase, SafetyPhase.FAULT)
        self.assertTrue(reconnected.current.brake_engaged)


if __name__ == "__main__":
    unittest.main()
