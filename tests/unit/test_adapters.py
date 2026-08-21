from __future__ import annotations

import sys
import time
from types import ModuleType, SimpleNamespace

import pytest

from car_interface.adapters.base import ConnectionFailed, TransportDisconnected
from car_interface.adapters.pygame_controller import PygameControllerSource
from car_interface.adapters.rplidar_source import RPLidarSource
from car_interface.adapters.serial_transport import SerialVehicleTransport


class FakeSerial:
    def __init__(self, responses: list[bytes] | None = None) -> None:
        self.is_open = True
        self.responses = list(responses or [])
        self.writes: list[bytes] = []
        self.input_resets = 0
        self.output_resets = 0
        self.closed = False

    def reset_input_buffer(self) -> None:
        self.input_resets += 1

    def reset_output_buffer(self) -> None:
        self.output_resets += 1

    def write(self, data: bytes) -> None:
        self.writes.append(data)

    def flush(self) -> None:
        return

    def read_until(self, *, expected: bytes, size: int) -> bytes:
        del expected, size
        return self.responses.pop(0) if self.responses else b""

    def close(self) -> None:
        self.closed = True
        self.is_open = False


def _install_fake_serial(monkeypatch: pytest.MonkeyPatch, fake: FakeSerial) -> None:
    module = ModuleType("serial")
    module.Serial = lambda *args, **kwargs: fake
    monkeypatch.setitem(sys.modules, "serial", module)


@pytest.mark.parametrize(
    ("args", "match"),
    [
        (("",), "port"),
        (("COM1", 0), "baud_rate"),
        (("COM1", True), "baud_rate"),
    ],
)
def test_serial_transport_rejects_invalid_construction(args, match: str) -> None:
    with pytest.raises(ValueError, match=match):
        SerialVehicleTransport(*args)
    with pytest.raises(ValueError, match="maximum_frame_bytes"):
        SerialVehicleTransport("COM1", maximum_frame_bytes=31)
    with pytest.raises(ValueError, match="startup_delay_seconds"):
        SerialVehicleTransport("COM1", startup_delay_seconds=float("nan"))


def test_serial_transport_connects_frames_transaction_and_disconnects(monkeypatch) -> None:
    fake = FakeSerial([b"!CAR,1,", b"ACK,0*FFFF\n"])
    _install_fake_serial(monkeypatch, fake)
    transport = SerialVehicleTransport(" COM7 ", startup_delay_seconds=0)

    transport.connect()
    assert transport.is_connected
    assert transport.description == "serial:COM7@115200"
    assert transport.transact("!CAR,1,CMD,0,HBT*FFFF", 0.1) == "!CAR,1,ACK,0*FFFF"
    assert fake.writes == [b"!CAR,1,CMD,0,HBT*FFFF\n"]
    assert fake.input_resets == 1
    assert fake.output_resets == 1

    transport.disconnect()
    transport.disconnect()
    assert fake.closed
    assert not transport.is_connected


@pytest.mark.parametrize(
    ("response", "match"),
    [
        (b"x" * 129, "maximum length"),
        (b"\xff\n", "ascii"),
    ],
)
def test_serial_transport_rejects_invalid_firmware_responses_and_closes_link(
    monkeypatch,
    response: bytes,
    match: str,
) -> None:
    fake = FakeSerial([response])
    _install_fake_serial(monkeypatch, fake)
    transport = SerialVehicleTransport("COM8", startup_delay_seconds=0)
    transport.connect()

    with pytest.raises(TransportDisconnected, match=match):
        transport.transact("frame", 0.1)

    assert fake.closed
    assert not transport.is_connected


def test_serial_transport_validates_frames_before_touching_disconnected_link() -> None:
    transport = SerialVehicleTransport("COM1", maximum_frame_bytes=32)
    with pytest.raises(ValueError, match="non-negative"):
        transport.transact("frame", -1)
    with pytest.raises(ValueError, match="line breaks"):
        transport.transact("bad\nframe", 0.1)
    with pytest.raises(ValueError, match="ASCII"):
        transport.transact("café", 0.1)
    with pytest.raises(ValueError, match="maximum length"):
        transport.transact("x" * 32, 0.1)
    with pytest.raises(TransportDisconnected, match="disconnected"):
        transport.transact("frame", 0.1)


def test_serial_transport_closes_link_on_incomplete_response(monkeypatch) -> None:
    fake = FakeSerial([b"partial"])
    _install_fake_serial(monkeypatch, fake)
    transport = SerialVehicleTransport("COM2", startup_delay_seconds=0)
    transport.connect()

    with pytest.raises(TransportDisconnected, match="incomplete"):
        transport.transact("frame", 0.001)

    assert fake.input_resets >= 2
    assert fake.closed
    assert not transport.is_connected


def test_serial_connect_failure_is_wrapped_and_closes_partial_connection(monkeypatch) -> None:
    fake = FakeSerial()
    fake.is_open = False
    _install_fake_serial(monkeypatch, fake)
    transport = SerialVehicleTransport("COM3", startup_delay_seconds=0)

    with pytest.raises(ConnectionFailed, match="could not open"):
        transport.connect()

    assert fake.closed
    assert not transport.is_connected


class FakeLidarDevice:
    def __init__(self, health: tuple[str, int] = ("Good", 0)) -> None:
        self.health = health
        self.operations: list[str] = []

    def get_info(self) -> dict[str, str]:
        return {"model": "test"}

    def get_health(self) -> tuple[str, int]:
        return self.health

    def iter_scans(self, **kwargs):
        assert kwargs == {"max_buf_meas": 5_000, "min_len": 5}
        yield [
            (12, 90.0, 1_000.0),
            (4, 180.0, 0.0),
            (8, 270.0, 50_000.0),
        ]

    def stop(self) -> None:
        self.operations.append("stop")

    def stop_motor(self) -> None:
        self.operations.append("stop_motor")

    def disconnect(self) -> None:
        self.operations.append("disconnect")


def _install_fake_lidar(monkeypatch: pytest.MonkeyPatch, device: FakeLidarDevice) -> None:
    module = ModuleType("rplidar")
    module.RPLidar = lambda *args, **kwargs: device
    monkeypatch.setitem(sys.modules, "rplidar", module)


def test_rplidar_normalizes_scan_and_cleans_up(monkeypatch) -> None:
    device = FakeLidarDevice()
    _install_fake_lidar(monkeypatch, device)
    source = RPLidarSource("COM9")
    source.connect()

    deadline = time.monotonic() + 1.0
    while source.latest_scan() is None and time.monotonic() < deadline:
        time.sleep(0.005)
    scan = source.latest_scan()
    assert scan is not None
    points, captured_at = scan
    assert captured_at > 0
    assert [(point.angle_degrees, point.distance_cm) for point in points] == [(0.0, 100.0)]

    source.disconnect()
    assert source.latest_scan() is None
    assert {"stop", "stop_motor", "disconnect"}.issubset(device.operations)


def test_rplidar_rejects_unhealthy_device_and_cleans_up(monkeypatch) -> None:
    device = FakeLidarDevice(("Error", 7))
    _install_fake_lidar(monkeypatch, device)
    source = RPLidarSource("COM10")

    with pytest.raises(ConnectionFailed, match="health"):
        source.connect()

    assert "disconnect" in device.operations
    assert not source.is_connected


class FakeJoystick:
    def __init__(self) -> None:
        self.initialized = False
        self.axes = {0: 0.4, 4: 0.6, 5: 0.5}

    def init(self) -> None:
        self.initialized = True

    def get_init(self) -> bool:
        return self.initialized

    def get_numaxes(self) -> int:
        return 6

    def get_name(self) -> str:
        return "Test Pad"

    def get_axis(self, axis: int) -> float:
        return self.axes[axis]

    def get_numhats(self) -> int:
        return 1

    def get_hat(self, hat: int) -> tuple[int, int]:
        assert hat == 0
        return (0, -1)

    def quit(self) -> None:
        self.initialized = False


def _fake_pygame(joystick: FakeJoystick, events: list[object]) -> ModuleType:
    module = ModuleType("pygame")
    module.JOYDEVICEREMOVED = 99
    module.init = lambda: None
    module.quit = lambda: None
    module.joystick = SimpleNamespace(
        init=lambda: None,
        quit=lambda: None,
        get_count=lambda: 1,
        Joystick=lambda controller_id: joystick,
    )
    module.event = SimpleNamespace(get=lambda: list(events))
    return module


def test_pygame_controller_normalizes_axes_direction_and_removal(monkeypatch) -> None:
    joystick = FakeJoystick()
    events: list[object] = []
    monkeypatch.setitem(sys.modules, "pygame", _fake_pygame(joystick, events))
    source = PygameControllerSource()
    source.connect()

    snapshot = source.poll()
    assert snapshot.connected
    assert snapshot.steering == pytest.approx(0.4)
    assert snapshot.throttle == pytest.approx(0.75)
    assert snapshot.brake_pressed
    assert snapshot.direction == "reverse"
    assert source.description == "controller:Test Pad"

    events.append(SimpleNamespace(type=99))
    assert not source.poll().connected
    assert not source.is_connected
    assert source.description == "controller:disconnected"


def test_pygame_controller_reports_missing_device_without_leaking_module(monkeypatch) -> None:
    joystick = FakeJoystick()
    module = _fake_pygame(joystick, [])
    module.joystick.get_count = lambda: 0
    monkeypatch.setitem(sys.modules, "pygame", module)
    source = PygameControllerSource(1)

    with pytest.raises(ConnectionFailed, match="not detected"):
        source.connect()

    assert not source.poll().connected
    assert source.description == "controller:disconnected"
