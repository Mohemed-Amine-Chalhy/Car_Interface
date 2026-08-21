"""Thread-safe service events consumed by the Tkinter main thread."""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass
from enum import StrEnum
from typing import Any


class EventType(StrEnum):
    STATE = "state"
    LOG = "log"
    SCAN = "scan"
    COMMAND = "command"
    FAULT = "fault"
    CONNECTION = "connection"


@dataclass(frozen=True, slots=True)
class ServiceEvent:
    type: EventType
    payload: Any
    created_at: float


class EventBroker:
    """A bounded UI event buffer; it never participates in safety decisions."""

    def __init__(self, maximum_events: int = 512) -> None:
        if maximum_events < 8:
            raise ValueError("maximum_events must be at least 8")
        self._events: deque[ServiceEvent] = deque(maxlen=maximum_events)
        self._lock = threading.Lock()

    def publish(self, event_type: EventType, payload: Any) -> None:
        event = ServiceEvent(event_type, payload, time.monotonic())
        with self._lock:
            if event_type == EventType.SCAN and self._events:
                # Only the newest scan matters to the visualizer.
                self._events = deque(
                    (item for item in self._events if item.type != EventType.SCAN),
                    maxlen=self._events.maxlen,
                )
            self._events.append(event)

    def drain(self, limit: int = 100) -> tuple[ServiceEvent, ...]:
        if limit <= 0:
            return ()
        drained: list[ServiceEvent] = []
        with self._lock:
            while self._events and len(drained) < limit:
                drained.append(self._events.popleft())
        return tuple(drained)
