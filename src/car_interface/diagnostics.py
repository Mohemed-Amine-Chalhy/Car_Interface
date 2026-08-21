"""Non-invasive environment diagnostics and support-bundle generation."""

from __future__ import annotations

import importlib.util
import json
import platform
import shutil
import sys
import zipfile
from dataclasses import asdict
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from . import __version__
from .config import AppConfig
from .logging_config import application_data_directory


def collect_diagnostics(config: AppConfig) -> dict[str, Any]:
    """Collect metadata without opening a controller, Lidar, or serial device."""

    modules = ("serial", "rplidar", "pygame", "tkinter")
    safe_config = asdict(config)
    safe_config["esp32_port"] = "<configured>" if config.esp32_port else None
    safe_config["lidar_port"] = "<configured>" if config.lidar_port else None
    safe_config["config_path"] = config.config_path.name if config.config_path else None
    return {
        "application_version": __version__,
        "created_at": datetime.now(UTC).isoformat(),
        "python": sys.version,
        "executable": Path(sys.executable).name,
        "platform": platform.platform(),
        "machine": platform.machine(),
        "configuration": safe_config,
        "dependencies_available": {
            module: importlib.util.find_spec(module) is not None for module in modules
        },
        "uv_available": shutil.which("uv") is not None,
    }


def write_support_bundle(config: AppConfig, destination: Path) -> Path:
    """Create a local ZIP containing diagnostics and bounded application logs."""

    destination = destination.resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    diagnostics = json.dumps(collect_diagnostics(config), indent=2, sort_keys=True)
    logs_directory = application_data_directory() / "logs"
    with zipfile.ZipFile(destination, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("diagnostics.json", diagnostics)
        if logs_directory.exists():
            for log_path in sorted(logs_directory.glob("car-interface.log*")):
                if log_path.is_file():
                    archive.write(log_path, f"logs/{log_path.name}")
    return destination
