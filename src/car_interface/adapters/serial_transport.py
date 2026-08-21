"""Exclusive pyserial transport for the ESP32."""

from __future__ import annotations

import logging
import threading
import time
from math import isfinite
from typing import Any

from .base import ConnectionFailed, TransportDisconnected

LOGGER = logging.getLogger(__name__)


class SerialVehicleTransport:
    """A small synchronous transport intended to be owned by one dispatcher."""

    def __init__(
        self,
        port: str,
        baud_rate: int = 115_200,
        *,
        maximum_frame_bytes: int = 128,
        startup_delay_seconds: float = 2.0,
    ) -> None:
        if not port.strip():
            raise ValueError("serial port cannot be empty")
        if isinstance(baud_rate, bool) or not isinstance(baud_rate, int) or baud_rate <= 0:
            raise ValueError("baud_rate must be a positive integer")
        if maximum_frame_bytes < 32:
            raise ValueError("maximum_frame_bytes must be at least 32")
        if not isfinite(startup_delay_seconds) or not 0 <= startup_delay_seconds <= 4.0:
            raise ValueError("startup_delay_seconds must be finite and between 0 and 4")
        self._port = port.strip()
        self._baud_rate = baud_rate
        self._maximum_frame_bytes = maximum_frame_bytes
        self._startup_delay_seconds = startup_delay_seconds
        self._serial: Any | None = None
        self._lock = threading.RLock()

    @property
    def is_connected(self) -> bool:
        with self._lock:
            return bool(self._serial is not None and self._serial.is_open)

    @property
    def description(self) -> str:
        return f"serial:{self._port}@{self._baud_rate}"

    def connect(self) -> None:
        with self._lock:
            if self.is_connected:
                return
        connection: Any | None = None
        try:
            import serial

            connection = serial.Serial(
                self._port,
                self._baud_rate,
                timeout=0.05,
                write_timeout=0.5,
            )
            with self._lock:
                self._serial = connection
            if self._startup_delay_seconds:
                time.sleep(self._startup_delay_seconds)
            with self._lock:
                if self._serial is not connection or not connection.is_open:
                    raise TransportDisconnected("serial connection closed during startup")
                connection.reset_input_buffer()
                connection.reset_output_buffer()
        except Exception as exc:
            with self._lock:
                if self._serial is connection:
                    self._serial = None
            if connection is not None:
                try:
                    connection.close()
                except Exception:
                    LOGGER.debug("Serial cleanup after startup failure failed", exc_info=True)
            raise ConnectionFailed(
                f"could not open ESP32 transport {self.description}: {exc}"
            ) from exc

    def transact(self, frame: str, timeout_seconds: float) -> str | None:
        if not isfinite(timeout_seconds) or timeout_seconds < 0:
            raise ValueError("timeout_seconds must be finite and non-negative")
        if "\n" in frame or "\r" in frame:
            raise ValueError("protocol frame must not contain line breaks")
        try:
            encoded = f"{frame}\n".encode("ascii")
        except UnicodeEncodeError as exc:
            raise ValueError("protocol frame must contain ASCII only") from exc
        if len(encoded) > self._maximum_frame_bytes:
            raise ValueError("protocol frame exceeds maximum length")
        with self._lock:
            if not self.is_connected or self._serial is None:
                raise TransportDisconnected(f"{self.description} is disconnected")
            try:
                self._serial.write(encoded)
                self._serial.flush()
                if timeout_seconds <= 0:
                    return None
                deadline = time.monotonic() + timeout_seconds
                response = bytearray()
                while time.monotonic() < deadline:
                    chunk = self._serial.read_until(
                        expected=b"\n",
                        size=(self._maximum_frame_bytes + 1) - len(response),
                    )
                    if chunk:
                        response.extend(chunk)
                        if len(response) > self._maximum_frame_bytes:
                            self._serial.reset_input_buffer()
                            raise ValueError("firmware response exceeds maximum length")
                        if response.endswith(b"\n"):
                            return bytes(response[:-1]).decode("ascii")
                if response:
                    self._serial.reset_input_buffer()
                    raise ValueError("firmware response was incomplete at the ACK deadline")
                return None
            except Exception as exc:
                self._close_unlocked()
                raise TransportDisconnected(
                    f"ESP32 transaction failed on {self.description}: {exc}"
                ) from exc

    def disconnect(self) -> None:
        with self._lock:
            self._close_unlocked()

    def _close_unlocked(self) -> None:
        connection, self._serial = self._serial, None
        if connection is not None:
            try:
                connection.close()
            except Exception:
                LOGGER.debug("Serial connection cleanup failed", exc_info=True)
