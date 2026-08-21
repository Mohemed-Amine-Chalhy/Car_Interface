"""Threaded RPLidar adapter with normalized centimetre measurements."""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any

from .base import ConnectionFailed, ScanPoint

LOGGER = logging.getLogger(__name__)


class RPLidarSource:
    """Own an RPLidar and expose an immutable latest-scan snapshot."""

    def __init__(
        self,
        port: str,
        *,
        front_angle_degrees: float = 90.0,
        max_distance_cm: float = 2_000.0,
    ) -> None:
        if not port.strip():
            raise ValueError("Lidar port cannot be empty")
        if not math.isfinite(front_angle_degrees):
            raise ValueError("front angle must be finite")
        if not math.isfinite(max_distance_cm) or max_distance_cm <= 0:
            raise ValueError("maximum distance must be finite and positive")
        self._port = port
        self._front_angle = front_angle_degrees
        self._max_distance = max_distance_cm
        self._lidar: Any | None = None
        self._scan_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._data_lock = threading.Lock()
        self._latest: tuple[tuple[ScanPoint, ...], float] | None = None
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected and not self._stop_event.is_set()

    @property
    def description(self) -> str:
        return f"rplidar:{self._port}"

    def connect(self) -> None:
        if self.is_connected:
            return
        try:
            from rplidar import RPLidar

            self._stop_event.clear()
            with self._data_lock:
                self._latest = None
            self._lidar = RPLidar(self._port, timeout=3)
            info = self._lidar.get_info()
            health = self._lidar.get_health()
            health_status = str(health[0]).lower() if health else "unknown"
            if health_status not in {"good", "ok"}:
                raise ConnectionFailed(f"Lidar health is not good: {health!r}")
            LOGGER.info("Connected to %s: %s", self.description, info)
            self._connected = True
            self._scan_thread = threading.Thread(
                target=self._scan_loop,
                name="rplidar-scan",
                daemon=True,
            )
            self._scan_thread.start()
        except Exception as exc:
            self._connected = False
            self._cleanup_device()
            if isinstance(exc, ConnectionFailed):
                raise
            raise ConnectionFailed(f"could not connect to {self.description}: {exc}") from exc

    def latest_scan(self) -> tuple[tuple[ScanPoint, ...], float] | None:
        with self._data_lock:
            return self._latest

    def disconnect(self) -> None:
        self._stop_event.set()
        self._connected = False
        device = self._lidar
        if device is not None:
            try:
                device.stop()
            except Exception:
                LOGGER.debug("Lidar stop request failed during cleanup", exc_info=True)
        thread = self._scan_thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        self._scan_thread = None
        self._cleanup_device()
        with self._data_lock:
            self._latest = None

    def _scan_loop(self) -> None:
        device = self._lidar
        if device is None:
            LOGGER.error("Lidar scan worker started without a device")
            self._stop_event.set()
            return
        try:
            for raw_scan in device.iter_scans(max_buf_meas=5_000, min_len=5):
                if self._stop_event.is_set():
                    break
                points: list[ScanPoint] = []
                for quality, sensor_angle, distance_mm in raw_scan:
                    distance_cm = float(distance_mm) / 10.0
                    if not 0.0 < distance_cm <= self._max_distance:
                        continue
                    robot_angle = (
                        (float(sensor_angle) - self._front_angle + 180.0) % 360.0
                    ) - 180.0
                    points.append(
                        ScanPoint(
                            angle_degrees=robot_angle,
                            distance_cm=distance_cm,
                            quality=int(quality),
                        )
                    )
                with self._data_lock:
                    self._latest = (tuple(points), time.monotonic())
        except Exception:
            if not self._stop_event.is_set():
                LOGGER.exception("Lidar scan worker failed")
        finally:
            self._connected = False
            self._stop_event.set()

    def _cleanup_device(self) -> None:
        device, self._lidar = self._lidar, None
        if device is None:
            return
        for operation_name in ("stop", "stop_motor", "disconnect"):
            try:
                operation = getattr(device, operation_name, None)
                if operation is not None:
                    operation()
            except Exception:
                LOGGER.debug("Lidar %s failed during cleanup", operation_name, exc_info=True)
