from __future__ import annotations

import threading
import time

from car_interface.adapters.simulated import SimulatedControllerSource, SimulatedLidarSource
from car_interface.config import AppConfig
from car_interface.domain import (
    CarV1WireProtocol,
    ControlCommand,
    ControlMode,
    LegacyProtocolContract,
    SchoolCarLegacyProtocol,
)
from car_interface.services.control import ControlService
from car_interface.services.dispatcher import CommandDispatcher


class WriteOnlyTransport:
    def __init__(self, *, fail_on_write: int | None = None) -> None:
        self.connected = False
        self.frames: list[str] = []
        self.write_times: list[float] = []
        self.fail_on_write = fail_on_write
        self._lock = threading.Lock()

    @property
    def is_connected(self) -> bool:
        return self.connected

    @property
    def description(self) -> str:
        return "test:legacy-write-only"

    def connect(self) -> None:
        self.connected = True

    def transact(self, frame: str, timeout_seconds: float) -> None:
        assert timeout_seconds == 0
        with self._lock:
            write_number = len(self.frames) + 1
            if write_number == self.fail_on_write:
                raise OSError("injected write failure")
            self.frames.append(frame)
            self.write_times.append(time.monotonic())

    def disconnect(self) -> None:
        self.connected = False


class AcknowledgingTransport(WriteOnlyTransport):
    def __init__(self) -> None:
        super().__init__()
        self.profile = CarV1WireProtocol()

    def transact(self, frame: str, timeout_seconds: float) -> str:
        assert timeout_seconds > 0
        self.frames.append(frame)
        decoded = self.profile.codec.parse_command(frame)
        return self.profile.codec.encode_ack(decoded.sequence).strip()


def test_legacy_profile_encodes_historical_commands_explicitly() -> None:
    profile = SchoolCarLegacyProtocol()

    assert profile.encode(ControlCommand.set_mode(ControlMode.MANUAL), None).frames == ("M",)
    assert profile.encode(ControlCommand.set_mode(ControlMode.AUTONOMOUS), None).frames == ("A",)
    assert profile.encode(ControlCommand.set_speed(40), None).frames == ("D F", "V 40")
    assert profile.encode(ControlCommand.set_speed(-35), None).frames == ("D R", "V 35")
    assert profile.encode(ControlCommand.set_speed(0), None).frames == ("V 0",)
    assert profile.encode(ControlCommand.set_brake(True), None).frames == ("S 1",)
    assert profile.encode(ControlCommand.set_brake(False), None).frames == ("Q 1",)
    assert profile.encode(ControlCommand.set_armed(True), None).frames == ("A",)
    assert profile.encode(ControlCommand.set_armed(False), None).frames == ("S 1", "M")
    assert profile.encode(ControlCommand.emergency_stop(), None).frames == ("V 0", "S 1")


def test_legacy_steering_uses_piecewise_calibration() -> None:
    profile = SchoolCarLegacyProtocol(
        LegacyProtocolContract(
            steering_minimum=200,
            steering_center=1_750,
            steering_maximum=2_900,
        )
    )

    assert profile.steering_raw(-100) == 200
    assert profile.steering_raw(-50) == 975
    assert profile.steering_raw(0) == 1_750
    assert profile.steering_raw(50) == 2_325
    assert profile.steering_raw(100) == 2_900
    assert profile.encode(ControlCommand.set_steering(-50), None).frames == ("W 975",)


def test_legacy_signed_speed_is_one_paced_write_only_operation() -> None:
    transport = WriteOnlyTransport()
    dispatcher = CommandDispatcher(
        transport,
        protocol=SchoolCarLegacyProtocol(
            LegacyProtocolContract(minimum_frame_interval_seconds=0.05)
        ),
        require_ack=False,
    )
    dispatcher.start()
    try:
        speed = dispatcher.submit(ControlCommand.set_speed(-25))
        steering = dispatcher.submit(ControlCommand.set_steering(0))

        assert speed.wait(1.0)
        assert steering.wait(1.0)
        assert transport.frames == ["D R", "V 25", "W 1750"]
        assert transport.write_times[1] - transport.write_times[0] >= 0.045
        assert speed.protocol_profile == "school_car_legacy_v0"
        assert speed.sequence is None
        assert speed.frames_total == 2
        assert speed.frames_written == 2
        assert speed.written
        assert not speed.acknowledged
        assert speed.response is None
    finally:
        dispatcher.stop()


def test_legacy_partial_write_is_not_reported_as_written_or_acknowledged() -> None:
    transport = WriteOnlyTransport(fail_on_write=2)
    dispatcher = CommandDispatcher(
        transport,
        protocol=SchoolCarLegacyProtocol(LegacyProtocolContract(minimum_frame_interval_seconds=0)),
        require_ack=False,
    )
    dispatcher.start()
    try:
        receipt = dispatcher.submit(ControlCommand.set_speed(25))

        assert not receipt.wait(1.0)
        assert receipt.done
        assert receipt.frames_total == 2
        assert receipt.frames_written == 1
        assert not receipt.written
        assert not receipt.acknowledged
        assert "injected write failure" in (receipt.error or "")
    finally:
        dispatcher.stop()


def test_legacy_heartbeat_is_handled_without_fabricating_a_write_or_ack() -> None:
    transport = WriteOnlyTransport()
    dispatcher = CommandDispatcher(
        transport,
        protocol=SchoolCarLegacyProtocol(LegacyProtocolContract(minimum_frame_interval_seconds=0)),
        require_ack=False,
    )
    dispatcher.start()
    try:
        receipt = dispatcher.submit(ControlCommand.heartbeat())

        assert receipt.wait(1.0)
        assert receipt.done
        assert receipt.frames_total == 0
        assert receipt.frames_written == 0
        assert not receipt.written
        assert not receipt.acknowledged
        assert transport.frames == []
    finally:
        dispatcher.stop()


def test_car_v1_profile_remains_sequence_and_acknowledgement_based() -> None:
    profile = CarV1WireProtocol()
    operation = profile.encode(ControlCommand.set_speed(10), 7)

    assert operation.sequence == 7
    assert operation.frames[0].startswith("!CAR,1,CMD,7,SPD,10*")
    assert profile.provides_acknowledgements
    assert profile.supports_heartbeat


def test_car_v1_dispatch_receipt_records_firmware_acknowledgement() -> None:
    transport = AcknowledgingTransport()
    dispatcher = CommandDispatcher(transport)
    dispatcher.start()
    try:
        receipt = dispatcher.submit(ControlCommand.set_speed(0))

        assert receipt.wait(1.0)
        assert receipt.written
        assert receipt.acknowledged
        assert receipt.sequence == 0
        assert receipt.protocol_profile == "car_v1"
    finally:
        dispatcher.stop()


def test_hardware_config_selects_legacy_dispatch_profile_without_auto_detection() -> None:
    config = AppConfig(
        mode="hardware",
        esp32_port="COM3",
        lidar_port="COM4",
        protocol="school_car_legacy_v0",
        require_ack=False,
        command_stale_seconds=0.1,
    )
    service = ControlService(
        config,
        vehicle=WriteOnlyTransport(),
        lidar=SimulatedLidarSource(),
        controller=SimulatedControllerSource(),
    )
    try:
        assert service._dispatcher.protocol_profile == "school_car_legacy_v0"
        assert not service._dispatcher.supports_heartbeat
    finally:
        service.shutdown()
