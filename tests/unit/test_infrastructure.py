from __future__ import annotations

import json
import logging
import zipfile
from pathlib import Path
from unittest.mock import Mock, patch

import pytest

from car_interface import diagnostics, factory, logging_config
from car_interface.adapters.simulated import (
    SimulatedControllerSource,
    SimulatedLidarSource,
    SimulatedVehicleTransport,
)
from car_interface.config import AppConfig
from car_interface.services.events import EventBroker, EventType


def test_event_broker_is_bounded_coalesces_scans_and_drains_in_order() -> None:
    with pytest.raises(ValueError, match="at least 8"):
        EventBroker(7)
    broker = EventBroker(maximum_events=8)
    broker.publish(EventType.LOG, "first")
    broker.publish(EventType.SCAN, "old scan")
    broker.publish(EventType.COMMAND, "second")
    broker.publish(EventType.SCAN, "latest scan")

    first_batch = broker.drain(limit=2)
    assert [(event.type, event.payload) for event in first_batch] == [
        (EventType.LOG, "first"),
        (EventType.COMMAND, "second"),
    ]
    final_batch = broker.drain()
    assert len(final_batch) == 1
    assert final_batch[0].payload == "latest scan"
    assert final_batch[0].created_at > 0
    assert broker.drain(limit=0) == ()

    for number in range(10):
        broker.publish(EventType.LOG, number)
    assert [event.payload for event in broker.drain()] == list(range(2, 10))


def test_collect_diagnostics_is_metadata_only_and_redacts_no_device_state(monkeypatch) -> None:
    monkeypatch.setattr(
        diagnostics.importlib.util,
        "find_spec",
        lambda name: object() if name == "serial" else None,
    )
    monkeypatch.setattr(diagnostics.shutil, "which", lambda name: f"/test/{name}")
    config = AppConfig(esp32_port="COM_LABEL", config_path=Path("settings.toml"))

    report = diagnostics.collect_diagnostics(config)

    assert report["application_version"]
    assert report["dependencies_available"]["serial"]
    assert not report["dependencies_available"]["pygame"]
    assert report["uv_available"]
    assert report["configuration"]["esp32_port"] == "<configured>"
    assert report["configuration"]["config_path"] == "settings.toml"


def test_support_bundle_contains_bounded_logs_and_diagnostics(tmp_path, monkeypatch) -> None:
    app_data = tmp_path / "app-data"
    logs = app_data / "logs"
    logs.mkdir(parents=True)
    (logs / "car-interface.log").write_text("safe log\n", encoding="utf-8")
    (logs / "unrelated.txt").write_text("exclude me\n", encoding="utf-8")
    monkeypatch.setattr(diagnostics, "application_data_directory", lambda: app_data)
    destination = tmp_path / "nested" / "support.zip"

    result = diagnostics.write_support_bundle(AppConfig(), destination)

    assert result == destination.resolve()
    with zipfile.ZipFile(result) as archive:
        assert set(archive.namelist()) == {
            "diagnostics.json",
            "logs/car-interface.log",
        }
        payload = json.loads(archive.read("diagnostics.json"))
        assert payload["configuration"]["mode"] == "simulation"


def test_logging_configuration_writes_to_selected_bounded_file(tmp_path) -> None:
    root = logging.getLogger()
    original_handlers = tuple(root.handlers)
    original_level = root.level
    try:
        log_path = logging_config.configure_logging("DEBUG", log_directory=tmp_path)
        logging.getLogger("car-interface-test").warning("bounded log test")
        for handler in root.handlers:
            handler.flush()
        assert log_path == tmp_path / "car-interface.log"
        assert "bounded log test" in log_path.read_text(encoding="utf-8")
        assert root.level == logging.DEBUG
    finally:
        for handler in tuple(root.handlers):
            root.removeHandler(handler)
            handler.close()
        root.setLevel(original_level)
        for handler in original_handlers:
            root.addHandler(handler)


def test_application_data_directory_uses_platform_local_state(monkeypatch, tmp_path) -> None:
    monkeypatch.setenv("LOCALAPPDATA", str(tmp_path))
    assert logging_config.application_data_directory() == tmp_path / "CarInterface"


def test_factory_composes_simulation_without_connecting_devices() -> None:
    service = factory.build_control_service(AppConfig())
    try:
        assert isinstance(service._vehicle, SimulatedVehicleTransport)
        assert isinstance(service._lidar, SimulatedLidarSource)
        assert isinstance(service._controller, SimulatedControllerSource)
        assert not service._vehicle.is_connected
        assert not service._lidar.is_connected
        assert not service._controller.is_connected
    finally:
        service.shutdown()


def test_factory_composes_hardware_adapters_without_opening_them() -> None:
    vehicle = Mock()
    lidar = Mock()
    controller = Mock()
    built_service = Mock()
    config = AppConfig(mode="hardware", esp32_port="COM1", lidar_port="COM2")
    with (
        patch.object(factory, "SerialVehicleTransport", return_value=vehicle) as serial,
        patch.object(factory, "RPLidarSource", return_value=lidar) as rplidar,
        patch.object(factory, "PygameControllerSource", return_value=controller) as pygame,
        patch.object(factory, "ControlService", return_value=built_service) as service,
    ):
        result = factory.build_control_service(config)

    assert result is built_service
    serial.assert_called_once_with("COM1", 115_200, startup_delay_seconds=2.0)
    rplidar.assert_called_once_with("COM2")
    pygame.assert_called_once_with(0)
    service.assert_called_once_with(config, vehicle=vehicle, lidar=lidar, controller=controller)
