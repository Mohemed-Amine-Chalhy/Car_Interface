from __future__ import annotations

import json
from pathlib import Path
from unittest.mock import Mock, patch

from car_interface import cli
from car_interface.config import AppConfig


def test_hardware_run_requires_explicit_acknowledgement_before_composition(
    capsys,
) -> None:
    with patch.object(cli, "build_control_service") as build_service:
        result = cli.main(
            [
                "run",
                "--mode",
                "hardware",
                "--esp32-port",
                "COM_TEST_ACTUATOR",
                "--lidar-port",
                "COM_TEST_LIDAR",
            ]
        )

    assert result == 2
    assert "requires --i-understand-this-controls-real-hardware" in capsys.readouterr().err
    build_service.assert_not_called()


def test_doctor_reports_environment_without_composing_or_opening_devices(capsys) -> None:
    report = {
        "python": "3.13.0 test",
        "platform": "test-platform",
        "dependencies_available": {
            "serial": True,
            "rplidar": True,
            "pygame": True,
            "tkinter": True,
        },
    }
    with (
        patch.object(cli, "collect_diagnostics", return_value=report) as collect,
        patch.object(cli, "_serial_ports", return_value=["COM_SAFE"]) as serial_ports,
        patch.object(cli, "build_control_service") as build_service,
    ):
        result = cli.main(["doctor", "--json"])

    assert result == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["serial_ports"] == ["COM_SAFE"]
    collect.assert_called_once_with(AppConfig())
    serial_ports.assert_called_once_with()
    build_service.assert_not_called()


def test_diagnostics_command_only_writes_requested_support_bundle(
    tmp_path: Path,
    capsys,
) -> None:
    destination = tmp_path / "support.zip"
    write_bundle = Mock(return_value=destination.resolve())
    with (
        patch.object(cli, "write_support_bundle", write_bundle),
        patch.object(cli, "build_control_service") as build_service,
    ):
        result = cli.main(["diagnostics", "--support-bundle", str(destination)])

    assert result == 0
    assert str(destination.resolve()) in capsys.readouterr().out
    write_bundle.assert_called_once_with(AppConfig(), destination)
    build_service.assert_not_called()
