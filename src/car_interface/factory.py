"""Composition root for simulated and real device adapters."""

from __future__ import annotations

from .adapters.pygame_controller import PygameControllerSource
from .adapters.rplidar_source import RPLidarSource
from .adapters.serial_transport import SerialVehicleTransport
from .adapters.simulated import (
    SimulatedControllerSource,
    SimulatedLidarSource,
    SimulatedVehicleTransport,
)
from .config import AppConfig, ConfigurationError
from .services.control import ControlService


def build_control_service(config: AppConfig) -> ControlService:
    """Build the selected runtime without connecting to any device."""

    config = config.validate()
    if config.mode == "simulation":
        return ControlService(
            config,
            vehicle=SimulatedVehicleTransport(config.command_stale_seconds),
            lidar=SimulatedLidarSource(),
            controller=SimulatedControllerSource(),
        )

    if config.esp32_port is None or config.lidar_port is None:
        raise ConfigurationError("validated hardware configuration is missing device ports")
    return ControlService(
        config,
        vehicle=SerialVehicleTransport(
            config.esp32_port,
            config.baud_rate,
            startup_delay_seconds=config.serial_startup_delay_seconds,
        ),
        lidar=RPLidarSource(config.lidar_port),
        controller=PygameControllerSource(
            config.controller_id,
            steering_invert=config.controller_steering_invert,
        ),
    )
