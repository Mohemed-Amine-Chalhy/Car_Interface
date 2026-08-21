from __future__ import annotations

import time

from hypothesis import given
from hypothesis import strategies as st

from car_interface.adapters.simulated import SimulatedVehicleTransport
from car_interface.domain import (
    CommandType,
    ControlCommand,
    ControlMode,
    ProtocolCodec,
    ResponseKind,
)


def _transact(
    transport: SimulatedVehicleTransport,
    codec: ProtocolCodec,
    command: ControlCommand,
    sequence: int,
):
    response = transport.transact(codec.encode_command(command, sequence), 0.1)
    assert response is not None
    return codec.parse_response(response, expected_sequence=sequence)


def test_unarmed_motion_is_nacked_and_cannot_change_safe_state() -> None:
    codec = ProtocolCodec()
    transport = SimulatedVehicleTransport()
    transport.connect()
    try:
        response = _transact(transport, codec, ControlCommand.set_speed(25), 0)
        assert response.kind is ResponseKind.NACK
        assert response.fault_code == "NOT_ARMED"
        assert transport.speed_percent == 0
        assert transport.braked
        assert not transport.armed
    finally:
        transport.disconnect()


def test_unarmed_nonzero_steering_is_nacked_without_changing_steering() -> None:
    codec = ProtocolCodec()
    transport = SimulatedVehicleTransport()
    transport.connect()
    try:
        response = _transact(transport, codec, ControlCommand.set_steering(40), 0)
        assert response.kind is ResponseKind.NACK
        assert response.fault_code == "NOT_ARMED"
        assert transport.steering == 0
        assert transport.braked

        neutral = _transact(transport, codec, ControlCommand.set_steering(0), 1)
        assert neutral.kind is ResponseKind.ACK
        assert transport.steering == 0
    finally:
        transport.disconnect()


def test_duplicate_sequence_replays_only_an_identical_cached_command() -> None:
    codec = ProtocolCodec()
    transport = SimulatedVehicleTransport()
    transport.connect()
    try:
        arm_frame = codec.encode_command(ControlCommand.set_armed(True), 0)
        first_response = transport.transact(arm_frame, 0.1)
        duplicate_response = transport.transact(arm_frame, 0.1)
        assert duplicate_response == first_response
        assert codec.parse_response(duplicate_response or "").kind is ResponseKind.ACK
        assert transport.armed

        conflicting_frame = codec.encode_command(ControlCommand.set_armed(False), 0)
        conflict = transport.transact(conflicting_frame, 0.1)
        assert conflict is not None
        parsed_conflict = codec.parse_response(conflict, expected_sequence=0)
        assert parsed_conflict.kind is ResponseKind.NACK
        assert parsed_conflict.fault_code == "BAD_SEQUENCE"
        assert transport.armed

        release = _transact(transport, codec, ControlCommand.set_brake(False), 1)
        assert release.kind is ResponseKind.ACK
        assert not transport.braked
    finally:
        transport.disconnect()


def test_out_of_order_sequence_is_rejected_without_advancing_expected_sequence() -> None:
    codec = ProtocolCodec()
    transport = SimulatedVehicleTransport()
    transport.connect()
    try:
        premature = _transact(transport, codec, ControlCommand.heartbeat(), 1)
        assert premature.kind is ResponseKind.NACK
        assert premature.fault_code == "BAD_SEQUENCE"

        expected = _transact(transport, codec, ControlCommand.heartbeat(), 0)
        following = _transact(transport, codec, ControlCommand.heartbeat(), 1)
        assert expected.kind is ResponseKind.ACK
        assert following.kind is ResponseKind.ACK
    finally:
        transport.disconnect()


def test_firmware_watchdog_returns_a_moving_vehicle_to_safe_state() -> None:
    codec = ProtocolCodec()
    transport = SimulatedVehicleTransport(watchdog_timeout_seconds=0.06)
    transport.connect()
    try:
        armed = _transact(transport, codec, ControlCommand.set_armed(True), 0)
        released = _transact(transport, codec, ControlCommand.set_brake(False), 1)
        assert armed.kind is ResponseKind.ACK
        assert released.kind is ResponseKind.ACK
        assert _transact(transport, codec, ControlCommand.set_speed(30), 2).kind is ResponseKind.ACK
        assert transport.speed_percent == 30

        deadline = time.monotonic() + 0.5
        while not transport.watchdog_tripped and time.monotonic() < deadline:
            time.sleep(0.005)
        assert transport.watchdog_tripped
        assert transport.speed_percent == 0
        assert transport.braked
        assert not transport.armed
    finally:
        transport.disconnect()


@given(
    sequence=st.integers(min_value=0, max_value=(2**31) - 1),
    command=st.one_of(
        st.integers(min_value=-100, max_value=100).map(ControlCommand.set_speed),
        st.integers(min_value=-100, max_value=100).map(ControlCommand.set_steering),
        st.booleans().map(ControlCommand.set_brake),
        st.booleans().map(ControlCommand.set_armed),
        st.sampled_from(tuple(ControlMode)).map(ControlCommand.set_mode),
        st.sampled_from(
            (
                ControlCommand(CommandType.HEARTBEAT),
                ControlCommand(CommandType.EMERGENCY_STOP),
                ControlCommand(CommandType.RESET),
            )
        ),
    ),
)
def test_protocol_round_trip_preserves_every_valid_bounded_command(
    sequence: int,
    command: ControlCommand,
) -> None:
    codec = ProtocolCodec()
    decoded = codec.parse_command(codec.encode_command(command, sequence))
    assert decoded.sequence == sequence
    assert decoded.command == command
