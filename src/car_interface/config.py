"""Typed application configuration loaded without device side effects."""

from __future__ import annotations

import os
import tomllib
from collections.abc import Mapping
from dataclasses import dataclass, fields, replace
from math import isfinite
from pathlib import Path
from typing import Any


class ConfigurationError(ValueError):
    """Configuration is missing or unsafe."""


@dataclass(frozen=True, slots=True)
class AppConfig:
    """Validated runtime settings.

    Simulation is deliberately the default. Real hardware also requires the
    explicit CLI safety acknowledgement; configuration alone cannot enable it.
    """

    mode: str = "simulation"
    esp32_port: str | None = None
    lidar_port: str | None = None
    baud_rate: int = 115_200
    serial_startup_delay_seconds: float = 2.0
    controller_id: int = 0
    require_ack: bool = True
    ack_timeout_seconds: float = 0.2
    heartbeat_interval_seconds: float = 0.1
    command_stale_seconds: float = 0.5
    lidar_stale_seconds: float = 0.75
    auto_stop_distance_cm: float = 50.0
    vehicle_width_cm: float = 42.0
    max_speed_percent: int = 50
    log_level: str = "INFO"
    config_path: Path | None = None

    def validate(self) -> AppConfig:
        if not isinstance(self.mode, str):
            raise ConfigurationError("mode must be a string")
        if not isinstance(self.log_level, str):
            raise ConfigurationError("log_level must be a string")
        for integer_name, integer_value in (
            ("baud_rate", self.baud_rate),
            ("controller_id", self.controller_id),
            ("max_speed_percent", self.max_speed_percent),
        ):
            if isinstance(integer_value, bool) or not isinstance(integer_value, int):
                raise ConfigurationError(f"{integer_name} must be an integer")
        for float_name, float_value in (
            ("ack_timeout_seconds", self.ack_timeout_seconds),
            ("serial_startup_delay_seconds", self.serial_startup_delay_seconds),
            ("heartbeat_interval_seconds", self.heartbeat_interval_seconds),
            ("command_stale_seconds", self.command_stale_seconds),
            ("lidar_stale_seconds", self.lidar_stale_seconds),
            ("auto_stop_distance_cm", self.auto_stop_distance_cm),
            ("vehicle_width_cm", self.vehicle_width_cm),
        ):
            if isinstance(float_value, bool) or not isinstance(float_value, int | float):
                raise ConfigurationError(f"{float_name} must be a number")
            if not isfinite(float_value):
                raise ConfigurationError(f"{float_name} must be finite")
        if not isinstance(self.require_ack, bool):
            raise ConfigurationError("require_ack must be a boolean")
        if self.mode not in {"simulation", "hardware"}:
            raise ConfigurationError("mode must be 'simulation' or 'hardware'")
        for port_name, port_value in (
            ("esp32_port", self.esp32_port),
            ("lidar_port", self.lidar_port),
        ):
            if port_value is not None and (
                not isinstance(port_value, str) or not port_value.strip()
            ):
                raise ConfigurationError(f"{port_name} must be a non-empty string")
        if self.mode == "hardware" and (
            not isinstance(self.esp32_port, str) or not self.esp32_port.strip()
        ):
            raise ConfigurationError("hardware mode requires esp32_port")
        if self.mode == "hardware" and (
            not isinstance(self.lidar_port, str) or not self.lidar_port.strip()
        ):
            raise ConfigurationError("hardware mode requires lidar_port")
        if self.mode == "hardware" and not self.require_ack:
            raise ConfigurationError("hardware mode requires protocol acknowledgements")
        if (
            self.mode == "hardware"
            and self.esp32_port is not None
            and self.lidar_port is not None
            and self.esp32_port.strip().casefold() == self.lidar_port.strip().casefold()
        ):
            raise ConfigurationError("esp32_port and lidar_port must be different devices")
        if not 1_200 <= self.baud_rate <= 4_000_000:
            raise ConfigurationError("baud_rate is outside the supported range")
        if self.controller_id < 0:
            raise ConfigurationError("controller_id cannot be negative")
        if not 0 <= self.serial_startup_delay_seconds <= 4.0:
            raise ConfigurationError("serial_startup_delay_seconds must be between 0 and 4")
        if not 0.05 <= self.ack_timeout_seconds <= 10.0:
            raise ConfigurationError("ack_timeout_seconds must be between 0.05 and 10")
        if not 0.05 <= self.heartbeat_interval_seconds <= 5.0:
            raise ConfigurationError("heartbeat_interval_seconds must be between 0.05 and 5")
        if self.command_stale_seconds <= (
            self.heartbeat_interval_seconds + self.ack_timeout_seconds
        ):
            raise ConfigurationError("command_stale_seconds must exceed heartbeat plus ACK timeout")
        if self.lidar_stale_seconds <= 0:
            raise ConfigurationError("lidar_stale_seconds must be positive")
        if not 5.0 <= self.auto_stop_distance_cm <= 500.0:
            raise ConfigurationError("auto_stop_distance_cm must be between 5 and 500 cm")
        if not 10.0 <= self.vehicle_width_cm <= 500.0:
            raise ConfigurationError("vehicle_width_cm must be between 10 and 500 cm")
        if not 1 <= self.max_speed_percent <= 100:
            raise ConfigurationError("max_speed_percent must be between 1 and 100")
        if self.log_level.upper() not in {
            "DEBUG",
            "INFO",
            "WARNING",
            "ERROR",
            "CRITICAL",
        }:
            raise ConfigurationError("log_level is invalid")
        return replace(
            self,
            esp32_port=self.esp32_port.strip() if self.esp32_port else None,
            lidar_port=self.lidar_port.strip() if self.lidar_port else None,
            log_level=self.log_level.upper(),
        )


_ENV_PREFIX = "CAR_INTERFACE_"
_BOOL_FIELDS = {"require_ack"}
_INT_FIELDS = {"baud_rate", "controller_id", "max_speed_percent"}
_FLOAT_FIELDS = {
    "serial_startup_delay_seconds",
    "ack_timeout_seconds",
    "heartbeat_interval_seconds",
    "command_stale_seconds",
    "lidar_stale_seconds",
    "auto_stop_distance_cm",
    "vehicle_width_cm",
}


def _coerce(name: str, value: Any) -> Any:
    if value is None:
        return None
    if name in _BOOL_FIELDS:
        if isinstance(value, bool):
            return value
        normalized = str(value).strip().lower()
        if normalized in {"1", "true", "yes", "on"}:
            return True
        if normalized in {"0", "false", "no", "off"}:
            return False
        raise ConfigurationError(f"{name} must be a boolean")
    if name in _INT_FIELDS:
        if isinstance(value, bool) or not isinstance(value, int | str):
            raise ConfigurationError(f"{name} must be an integer")
        try:
            return int(value)
        except ValueError as exc:
            raise ConfigurationError(f"{name} must be an integer") from exc
    if name in _FLOAT_FIELDS:
        if isinstance(value, bool) or not isinstance(value, int | float | str):
            raise ConfigurationError(f"{name} must be a number")
        try:
            return float(value)
        except ValueError as exc:
            raise ConfigurationError(f"{name} must be a number") from exc
    if not isinstance(value, str):
        raise ConfigurationError(f"{name} must be a string")
    return value


def _known_values(values: Mapping[str, Any]) -> dict[str, Any]:
    known_names = {field.name for field in fields(AppConfig)} - {"config_path"}
    unknown = sorted(set(values) - known_names)
    if unknown:
        raise ConfigurationError(f"unknown configuration keys: {', '.join(unknown)}")
    return {name: _coerce(name, value) for name, value in values.items()}


def load_config(
    config_file: Path | None = None,
    *,
    environ: Mapping[str, str] | None = None,
    overrides: Mapping[str, Any] | None = None,
) -> AppConfig:
    """Load defaults, TOML, environment, then explicit overrides."""

    values: dict[str, Any] = {}
    if config_file is not None:
        try:
            with config_file.open("rb") as handle:
                document = tomllib.load(handle)
        except (OSError, tomllib.TOMLDecodeError) as exc:
            raise ConfigurationError(f"cannot read configuration {config_file}: {exc}") from exc
        if "car_interface" in document:
            unexpected_sections = sorted(set(document) - {"car_interface"})
            if unexpected_sections:
                raise ConfigurationError(
                    "unknown top-level TOML keys: " + ", ".join(unexpected_sections)
                )
            section = document["car_interface"]
        else:
            section = document
        if not isinstance(section, dict):
            raise ConfigurationError("TOML car_interface section must be a table")
        values.update(_known_values(section))
        values["config_path"] = config_file.resolve()

    env = os.environ if environ is None else environ
    for field in fields(AppConfig):
        if field.name == "config_path":
            continue
        key = f"{_ENV_PREFIX}{field.name.upper()}"
        if key in env:
            values[field.name] = _coerce(field.name, env[key])

    if overrides:
        values.update(
            _known_values({key: value for key, value in overrides.items() if value is not None})
        )

    return AppConfig(**values).validate()
