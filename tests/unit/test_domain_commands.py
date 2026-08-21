from __future__ import annotations

import unittest

from car_interface.domain import (
    MAX_SEQUENCE,
    CommandPriority,
    CommandType,
    CommandValidationError,
    ControlCommand,
    ControlMode,
    FaultCode,
    ProtocolCodec,
    ProtocolContract,
    ProtocolError,
    ResponseKind,
    crc16_ccitt,
)


class ControlCommandTests(unittest.TestCase):
    def test_factories_create_typed_commands(self) -> None:
        self.assertEqual(ControlCommand.set_speed(-25).value, -25)
        self.assertEqual(ControlCommand.set_steering(80).kind, CommandType.SET_STEERING)
        self.assertEqual(ControlCommand.set_brake(True).value, True)
        self.assertEqual(
            ControlCommand.set_mode(ControlMode.AUTONOMOUS).value,
            ControlMode.AUTONOMOUS,
        )
        self.assertEqual(ControlCommand.set_armed(False).kind, CommandType.SET_ARMED)
        self.assertIsNone(ControlCommand.heartbeat().value)

    def test_percent_ranges_are_closed_and_boolean_is_not_an_integer(self) -> None:
        for value in (-100, 0, 100):
            self.assertEqual(ControlCommand.set_speed(value).value, value)
            self.assertEqual(ControlCommand.set_steering(value).value, value)
        for value in (-101, 101, True):
            with self.subTest(value=value), self.assertRaises(CommandValidationError):
                ControlCommand.set_speed(value)  # type: ignore[arg-type]

    def test_command_value_types_are_enforced(self) -> None:
        invalid = (
            (CommandType.SET_BRAKE, 1),
            (CommandType.SET_ARMED, None),
            (CommandType.SET_MODE, "manual"),
            (CommandType.HEARTBEAT, 1),
        )
        for kind, value in invalid:
            with self.subTest(kind=kind), self.assertRaises(CommandValidationError):
                ControlCommand(kind, value)  # type: ignore[arg-type]

    def test_priorities_put_emergency_and_safety_before_motion(self) -> None:
        self.assertEqual(ControlCommand.emergency_stop().priority, CommandPriority.EMERGENCY)
        self.assertEqual(ControlCommand.set_speed(0).priority, CommandPriority.SAFETY)
        self.assertEqual(ControlCommand.set_brake(True).priority, CommandPriority.SAFETY)
        self.assertEqual(ControlCommand.set_speed(1).priority, CommandPriority.CONTROL)
        self.assertEqual(ControlCommand.heartbeat().priority, CommandPriority.BACKGROUND)


class ProtocolCodecTests(unittest.TestCase):
    def setUp(self) -> None:
        self.codec = ProtocolCodec()

    def test_crc_uses_standard_ccitt_false_parameters(self) -> None:
        self.assertEqual(crc16_ccitt(b"123456789"), 0x29B1)

    def test_all_commands_round_trip_and_frames_end_in_newline(self) -> None:
        commands = (
            ControlCommand.set_speed(-100),
            ControlCommand.set_steering(100),
            ControlCommand.set_brake(True),
            ControlCommand.set_mode(ControlMode.MANUAL),
            ControlCommand.set_armed(False),
            ControlCommand.heartbeat(),
            ControlCommand.emergency_stop(),
            ControlCommand.reset(),
        )
        for sequence, command in enumerate(commands):
            with self.subTest(command=command):
                frame = self.codec.encode_command(command, sequence)
                self.assertTrue(frame.endswith("\n"))
                self.assertTrue(frame.startswith("!CAR,1,CMD,"))
                decoded = self.codec.parse_command(frame)
                self.assertEqual(decoded.sequence, sequence)
                self.assertEqual(decoded.command, command)
                self.assertEqual(
                    self.codec.encode_command_bytes(command, sequence),
                    frame.encode("ascii"),
                )

    def test_parser_accepts_transport_stripped_line(self) -> None:
        frame = self.codec.encode_command(ControlCommand.set_speed(5), 9).rstrip("\n")
        self.assertEqual(self.codec.parse_command(frame).command.value, 5)

    def test_ack_and_nack_round_trip_with_expected_sequence(self) -> None:
        ack = self.codec.parse_response(self.codec.encode_ack(7), expected_sequence=7)
        self.assertEqual(ack.kind, ResponseKind.ACK)
        self.assertIsNone(ack.fault_code)

        nack = self.codec.parse_response(
            self.codec.encode_nack(8, FaultCode.WATCHDOG_TIMEOUT),
            expected_sequence=8,
        )
        self.assertEqual(nack.kind, ResponseKind.NACK)
        self.assertEqual(nack.fault_code, "WATCHDOG_TIMEOUT")

    def test_wrong_response_sequence_is_rejected(self) -> None:
        with self.assertRaisesRegex(ProtocolError, "unexpected response sequence"):
            self.codec.parse_response(self.codec.encode_ack(1), expected_sequence=2)

    def test_corruption_bad_framing_and_unsupported_version_are_rejected(self) -> None:
        valid = self.codec.encode_command(ControlCommand.set_speed(25), 1)
        corrupt = valid.replace(",25*", ",24*")
        with self.assertRaisesRegex(ProtocolError, "checksum mismatch"):
            self.codec.parse_command(corrupt)
        with self.assertRaises(ProtocolError):
            self.codec.parse_command(valid.replace("!CAR", "CAR", 1))

        v2 = ProtocolCodec(ProtocolContract(version=2)).encode_command(
            ControlCommand.heartbeat(), 2
        )
        with self.assertRaisesRegex(ProtocolError, "unsupported protocol version"):
            self.codec.parse_command(v2)

    def test_noncanonical_values_are_rejected_even_with_a_valid_crc(self) -> None:
        noncanonical = self.codec._frame(["CAR", "1", "CMD", "01", "SPD", "01"])
        with self.assertRaisesRegex(ProtocolError, "canonical"):
            self.codec.parse_command(noncanonical)

    def test_sequence_range_and_frame_contract_are_validated(self) -> None:
        for sequence in (-1, MAX_SEQUENCE + 1, True):
            with self.subTest(sequence=sequence), self.assertRaises((TypeError, ValueError)):
                self.codec.encode_ack(sequence)  # type: ignore[arg-type]
        with self.assertRaises(ValueError):
            ProtocolContract(watchdog_timeout_seconds=0.25)


if __name__ == "__main__":
    unittest.main()
