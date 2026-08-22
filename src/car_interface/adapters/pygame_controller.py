"""Pygame controller adapter with normalized axes and edge-triggered actions."""

from __future__ import annotations

import logging
import math
from typing import Any

from .base import ConnectionFailed, ControllerSnapshot

LOGGER = logging.getLogger(__name__)


class PygameControllerSource:
    """Read one Xbox/PlayStation-compatible controller without UI coupling."""

    def __init__(self, controller_id: int = 0, *, steering_invert: bool = False) -> None:
        if controller_id < 0:
            raise ValueError("controller_id cannot be negative")
        if not isinstance(steering_invert, bool):
            raise TypeError("steering_invert must be a boolean")
        self._controller_id = controller_id
        self._steering_multiplier = -1.0 if steering_invert else 1.0
        self._pygame: Any | None = None
        self._joystick: Any | None = None
        self._name = "controller:disconnected"
        self._direction = "forward"

    @property
    def is_connected(self) -> bool:
        try:
            return bool(self._joystick is not None and self._joystick.get_init())
        except Exception:
            LOGGER.debug("Controller status probe failed", exc_info=True)
            return False

    @property
    def description(self) -> str:
        return self._name

    def connect(self) -> None:
        if self.is_connected:
            return
        try:
            import pygame

            self._pygame = pygame
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() <= self._controller_id:
                raise ConnectionFailed(f"controller {self._controller_id} was not detected")
            self._joystick = pygame.joystick.Joystick(self._controller_id)
            self._joystick.init()
            if self._joystick.get_numaxes() <= 5:
                raise ConnectionFailed(
                    "controller does not expose the required steering and trigger axes"
                )
            self._name = f"controller:{self._joystick.get_name()}"
        except Exception as exc:
            self.disconnect()
            if isinstance(exc, ConnectionFailed):
                raise
            raise ConnectionFailed(f"could not initialize controller: {exc}") from exc

    def poll(self) -> ControllerSnapshot:
        if not self.is_connected or self._pygame is None or self._joystick is None:
            return ControllerSnapshot(connected=False)
        try:
            for event in self._pygame.event.get():
                if event.type == self._pygame.JOYDEVICEREMOVED:
                    self.disconnect()
                    return ControllerSnapshot(connected=False)

            axes = self._joystick.get_numaxes()
            steering_raw = (
                self._joystick.get_axis(0) * self._steering_multiplier if axes > 0 else 0.0
            )
            rt_raw = self._joystick.get_axis(5) if axes > 5 else -1.0
            lt_raw = self._joystick.get_axis(4) if axes > 4 else -1.0
            if not all(math.isfinite(value) for value in (steering_raw, rt_raw, lt_raw)):
                raise ValueError("controller returned a non-finite axis value")
            steering = 0.0 if abs(steering_raw) < 0.15 else float(steering_raw)
            throttle = max(0.0, min(1.0, (float(rt_raw) + 1.0) / 2.0))
            if throttle < 0.05:
                throttle = 0.0
            brake = float(lt_raw) > 0.5

            if self._joystick.get_numhats() > 0:
                _, vertical = self._joystick.get_hat(0)
                if vertical > 0:
                    self._direction = "forward"
                elif vertical < 0:
                    self._direction = "reverse"

            return ControllerSnapshot(
                connected=True,
                steering=max(-1.0, min(1.0, steering)),
                throttle=throttle,
                brake_pressed=brake,
                direction=self._direction,
            )
        except Exception:
            LOGGER.warning(
                "Controller poll failed; treating the controller as disconnected", exc_info=True
            )
            self.disconnect()
            return ControllerSnapshot(connected=False)

    def disconnect(self) -> None:
        joystick, self._joystick = self._joystick, None
        if joystick is not None:
            try:
                joystick.quit()
            except Exception:
                LOGGER.debug("Controller joystick cleanup failed", exc_info=True)
        pygame, self._pygame = self._pygame, None
        if pygame is not None:
            try:
                pygame.joystick.quit()
                pygame.quit()
            except Exception:
                LOGGER.debug("Pygame cleanup failed", exc_info=True)
        self._name = "controller:disconnected"
        self._direction = "forward"
