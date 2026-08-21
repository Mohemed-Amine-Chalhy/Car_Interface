from __future__ import annotations

from pathlib import Path

import pytest

from car_interface.config import AppConfig, ConfigurationError, load_config


def test_named_config_table_rejects_every_other_top_level_section(tmp_path: Path) -> None:
    config_path = tmp_path / "config.toml"
    config_path.write_text(
        "[car_interface]\nmax_speed_percent = 25\n\n[unexpected]\nenabled = true\n",
        encoding="utf-8",
    )

    with pytest.raises(ConfigurationError, match="unknown top-level TOML keys: unexpected"):
        load_config(config_path, environ={})


def test_serial_startup_delay_loads_as_a_finite_numeric_environment_value() -> None:
    config = load_config(environ={"CAR_INTERFACE_SERIAL_STARTUP_DELAY_SECONDS": "0.25"})

    assert config.serial_startup_delay_seconds == pytest.approx(0.25)


@pytest.mark.parametrize("value", [-0.01, 4.01, float("inf"), float("nan"), True])
def test_serial_startup_delay_rejects_unsafe_values(value: object) -> None:
    with pytest.raises(ConfigurationError, match="serial_startup_delay_seconds"):
        AppConfig(serial_startup_delay_seconds=value).validate()


def test_hardware_ports_are_trimmed_and_must_resolve_to_different_devices() -> None:
    with pytest.raises(ConfigurationError, match="must be different devices"):
        AppConfig(
            mode="hardware",
            esp32_port=" COM7 ",
            lidar_port="com7",
        ).validate()

    config = AppConfig(
        mode="hardware",
        esp32_port=" COM7 ",
        lidar_port=" COM8 ",
        serial_startup_delay_seconds=0,
    ).validate()
    assert config.esp32_port == "COM7"
    assert config.lidar_port == "COM8"
