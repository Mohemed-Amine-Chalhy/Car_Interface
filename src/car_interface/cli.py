"""Command-line entry point for operation and non-invasive diagnostics."""

from __future__ import annotations

import argparse
import json
import logging
import sys
from collections.abc import Sequence
from pathlib import Path
from typing import TYPE_CHECKING, Any

from .config import AppConfig, ConfigurationError, load_config
from .diagnostics import collect_diagnostics, write_support_bundle
from .factory import build_control_service
from .logging_config import configure_logging

if TYPE_CHECKING:
    import argparse

LOGGER = logging.getLogger(__name__)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="car-interface",
        description="Safe ESP32 car controller (simulation by default)",
    )
    parser.add_argument("--version", action="version", version=_version_string())
    subparsers = parser.add_subparsers(dest="command", required=True)

    run = subparsers.add_parser("run", help="start the desktop application")
    _add_config_argument(run)
    run.add_argument("--mode", choices=("simulation", "hardware"))
    run.add_argument("--esp32-port")
    run.add_argument("--lidar-port")
    run.add_argument("--controller-id", type=int)
    run.add_argument("--protocol", choices=("car_v1", "school_car_legacy_v0"))
    run.add_argument("--max-speed-percent", type=int)
    run.add_argument("--log-level", choices=("DEBUG", "INFO", "WARNING", "ERROR"))
    run.add_argument(
        "--i-understand-this-controls-real-hardware",
        action="store_true",
        help="required acknowledgement before hardware adapters can be composed",
    )

    doctor = subparsers.add_parser("doctor", help="inspect the environment without opening devices")
    _add_config_argument(doctor)
    doctor.add_argument("--json", action="store_true", dest="as_json")

    diagnostics = subparsers.add_parser(
        "diagnostics", help="write a local support bundle without opening devices"
    )
    _add_config_argument(diagnostics)
    diagnostics.add_argument(
        "--support-bundle",
        type=Path,
        required=True,
        metavar="PATH",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "run":
            return _run(args)
        config = _load_from_args(args)
        if args.command == "doctor":
            return _doctor(config, as_json=args.as_json)
        if args.command == "diagnostics":
            bundle = write_support_bundle(config, args.support_bundle)
            print(bundle)
            return 0
    except ConfigurationError as exc:
        print(f"Configuration error: {exc}", file=sys.stderr)
        return 2
    except Exception as exc:
        LOGGER.exception("Command failed")
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    return 2


def _run(args: argparse.Namespace) -> int:
    import tkinter as tk

    from .ui import CarInterfaceWindow

    config = _load_from_args(args)
    if config.mode == "hardware" and not args.i_understand_this_controls_real_hardware:
        raise ConfigurationError(
            "hardware mode requires --i-understand-this-controls-real-hardware"
        )
    log_path = configure_logging(config.log_level)
    LOGGER.info("Starting Car Interface in %s mode; logs: %s", config.mode, log_path)
    service = build_control_service(config)
    root = tk.Tk()
    CarInterfaceWindow(
        root,
        service,
        simulation=config.mode == "simulation",
        max_speed_percent=config.max_speed_percent,
    )
    try:
        root.mainloop()
    finally:
        service.shutdown()
    return 0


def _doctor(config: AppConfig, *, as_json: bool) -> int:
    report = collect_diagnostics(config)
    report["serial_ports"] = _serial_ports()
    if as_json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(f"Car Interface {_version_string()}")
        print(f"Python: {report['python'].splitlines()[0]}")
        print(f"Platform: {report['platform']}")
        print(f"Mode: {config.mode}")
        dependencies = report["dependencies_available"]
        for name, available in dependencies.items():
            print(f"Dependency {name}: {'available' if available else 'missing'}")
        ports = report["serial_ports"]
        print("Serial ports: " + (", ".join(ports) if ports else "none detected"))
    required = (
        ("serial", "rplidar", "pygame", "tkinter") if config.mode == "hardware" else ("tkinter",)
    )
    missing = [name for name in required if not report["dependencies_available"][name]]
    return 1 if missing else 0


def _load_from_args(args: argparse.Namespace) -> AppConfig:
    config_path: Path | None = getattr(args, "config", None)
    overrides: dict[str, Any] = {}
    for name in (
        "mode",
        "esp32_port",
        "lidar_port",
        "controller_id",
        "protocol",
        "max_speed_percent",
        "log_level",
    ):
        value = getattr(args, name, None)
        if value is not None:
            overrides[name] = value
    return load_config(config_path, overrides=overrides)


def _serial_ports() -> list[str]:
    try:
        from serial.tools import list_ports

        return sorted(port.device for port in list_ports.comports())
    except ImportError:
        return []


def _add_config_argument(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--config", type=Path, help="path to a TOML configuration file")


def _version_string() -> str:
    from . import __version__

    return __version__
