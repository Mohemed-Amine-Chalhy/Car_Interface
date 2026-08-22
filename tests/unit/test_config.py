from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from car_interface.config import AppConfig, ConfigurationError, load_config


class AppConfigTests(unittest.TestCase):
    def test_simulation_is_safe_default(self) -> None:
        config = AppConfig().validate()

        self.assertEqual(config.mode, "simulation")
        self.assertEqual(config.protocol, "car_v1")
        self.assertTrue(config.require_ack)
        self.assertIsNone(config.esp32_port)

    def test_hardware_requires_ports_and_acknowledgements(self) -> None:
        with self.assertRaisesRegex(ConfigurationError, "esp32_port"):
            AppConfig(mode="hardware").validate()
        with self.assertRaisesRegex(ConfigurationError, "lidar_port"):
            AppConfig(mode="hardware", esp32_port="COM3").validate()
        with self.assertRaisesRegex(ConfigurationError, "acknowledgements"):
            AppConfig(
                mode="hardware",
                esp32_port="COM3",
                lidar_port="COM4",
                require_ack=False,
            ).validate()

    def test_timing_contract_prevents_watchdog_overlap(self) -> None:
        with self.assertRaisesRegex(ConfigurationError, "heartbeat plus ACK"):
            AppConfig(
                ack_timeout_seconds=0.2,
                heartbeat_interval_seconds=0.2,
                command_stale_seconds=0.4,
            ).validate()

    def test_legacy_profile_requires_truthful_write_only_configuration(self) -> None:
        with self.assertRaisesRegex(ConfigurationError, "only in explicit hardware mode"):
            AppConfig(protocol="school_car_legacy_v0").validate()
        with self.assertRaisesRegex(ConfigurationError, "has no acknowledgements"):
            AppConfig(
                mode="hardware",
                esp32_port="COM3",
                lidar_port="COM4",
                protocol="school_car_legacy_v0",
            ).validate()

        config = AppConfig(
            mode="hardware",
            esp32_port="COM3",
            lidar_port="COM4",
            protocol="school_car_legacy_v0",
            require_ack=False,
            controller_steering_invert=True,
        ).validate()

        self.assertEqual(config.protocol, "school_car_legacy_v0")
        self.assertFalse(config.require_ack)
        self.assertTrue(config.controller_steering_invert)
        self.assertEqual(config.legacy_minimum_command_interval_ms, 50)

    def test_legacy_calibration_is_ordered_and_bounded(self) -> None:
        with self.assertRaisesRegex(ConfigurationError, "legacy steering calibration"):
            AppConfig(legacy_steering_minimum=2_000).validate()
        with self.assertRaisesRegex(ConfigurationError, "interval"):
            AppConfig(legacy_minimum_command_interval_ms=0).validate()

    def test_load_precedence_is_defaults_file_environment_overrides(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.toml"
            path.write_text(
                "[car_interface]\nmax_speed_percent = 30\nlog_level = 'WARNING'\n",
                encoding="utf-8",
            )
            config = load_config(
                path,
                environ={"CAR_INTERFACE_MAX_SPEED_PERCENT": "40"},
                overrides={"max_speed_percent": 45},
            )

        self.assertEqual(config.max_speed_percent, 45)
        self.assertEqual(config.log_level, "WARNING")
        self.assertEqual(config.config_path, path.resolve())

    def test_legacy_profile_fields_load_from_environment(self) -> None:
        config = load_config(
            environ={
                "CAR_INTERFACE_MODE": "hardware",
                "CAR_INTERFACE_ESP32_PORT": "COM3",
                "CAR_INTERFACE_LIDAR_PORT": "COM4",
                "CAR_INTERFACE_PROTOCOL": "school_car_legacy_v0",
                "CAR_INTERFACE_REQUIRE_ACK": "false",
                "CAR_INTERFACE_CONTROLLER_STEERING_INVERT": "yes",
                "CAR_INTERFACE_LEGACY_STEERING_CENTER": "1800",
                "CAR_INTERFACE_LEGACY_MINIMUM_COMMAND_INTERVAL_MS": "60",
            }
        )

        self.assertEqual(config.protocol, "school_car_legacy_v0")
        self.assertFalse(config.require_ack)
        self.assertTrue(config.controller_steering_invert)
        self.assertEqual(config.legacy_steering_center, 1_800)
        self.assertEqual(config.legacy_minimum_command_interval_ms, 60)

    def test_unknown_configuration_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.toml"
            path.write_text("[car_interface]\nunsafe_magic = true\n", encoding="utf-8")
            with self.assertRaisesRegex(ConfigurationError, "unknown configuration"):
                load_config(path, environ={})


if __name__ == "__main__":
    unittest.main()
