"""Single-owner, prioritized command dispatcher for the actuator transport."""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from itertools import count

from car_interface.adapters.base import AdapterError, VehicleTransport
from car_interface.domain import (
    MAX_SEQUENCE,
    CommandPriority,
    CommandType,
    ControlCommand,
    ProtocolCodec,
    ProtocolError,
    ResponseKind,
    SafetyDecision,
)

LOGGER = logging.getLogger(__name__)


@dataclass(slots=True)
class DispatchReceipt:
    """Observable result of one queued actuator command."""

    command: ControlCommand
    sequence: int | None = None
    succeeded: bool = False
    cancelled: bool = False
    response: str | None = None
    error: str | None = None
    _finished: threading.Event = field(default_factory=threading.Event, repr=False)

    @property
    def done(self) -> bool:
        return self._finished.is_set()

    def wait(self, timeout_seconds: float | None = None) -> bool:
        """Wait for completion and return whether the command was acknowledged."""

        self._finished.wait(timeout_seconds)
        return self.done and self.succeeded

    def _complete(
        self,
        *,
        succeeded: bool = False,
        cancelled: bool = False,
        response: str | None = None,
        error: str | None = None,
    ) -> None:
        self.succeeded = succeeded
        self.cancelled = cancelled
        self.response = response
        self.error = error
        self._finished.set()


@dataclass(slots=True)
class _PendingCommand:
    priority: int
    order: int
    command: ControlCommand
    receipt: DispatchReceipt
    enqueued_at: float


ReceiptCallback = Callable[[DispatchReceipt], None]


class CommandDispatcher:
    """Own the transport and guarantee safety-first bounded dispatch.

    Motion requests are coalesced, so controller sampling can never build an
    unbounded backlog. A decision marked ``discard_pending_commands`` atomically
    cancels everything queued before inserting its safe-stop commands.
    """

    def __init__(
        self,
        transport: VehicleTransport,
        *,
        codec: ProtocolCodec | None = None,
        require_ack: bool = True,
        ack_timeout_seconds: float = 0.2,
        maximum_command_age_seconds: float = 0.5,
        maximum_pending: int = 128,
        on_receipt: ReceiptCallback | None = None,
    ) -> None:
        if maximum_pending < 8:
            raise ValueError("maximum_pending must be at least 8")
        if maximum_command_age_seconds <= 0:
            raise ValueError("maximum_command_age_seconds must be positive")
        self._transport = transport
        self._codec = codec or ProtocolCodec()
        self._require_ack = require_ack
        self._ack_timeout = ack_timeout_seconds
        self._maximum_command_age = maximum_command_age_seconds
        self._maximum_pending = maximum_pending
        self._on_receipt = on_receipt
        self._condition = threading.Condition()
        self._pending: list[_PendingCommand] = []
        self._order = count()
        self._sequence = 0
        self._thread: threading.Thread | None = None
        self._running = False
        self._started = threading.Event()
        self._start_error: str | None = None
        self._active = False

    @property
    def is_running(self) -> bool:
        with self._condition:
            return self._running and self._thread is not None and self._thread.is_alive()

    @property
    def transport_description(self) -> str:
        return self._transport.description

    def start(self, timeout_seconds: float = 5.0) -> None:
        with self._condition:
            current_thread = self._thread
            if current_thread is not None and current_thread.is_alive():
                if self._running:
                    return
                raise RuntimeError("the previous actuator dispatcher is still stopping")
            self._sequence = 0
            self._order = count()
            self._running = True
            self._started.clear()
            self._start_error = None
            self._thread = threading.Thread(
                target=self._worker,
                name="actuator-dispatch",
                daemon=True,
            )
            self._thread.start()
        if not self._started.wait(timeout_seconds):
            self.stop()
            raise TimeoutError("actuator dispatcher did not start in time")
        if self._start_error is not None:
            raise AdapterError(self._start_error)

    def submit(
        self,
        command: ControlCommand,
        *,
        discard_pending: bool = False,
    ) -> DispatchReceipt:
        receipt = DispatchReceipt(command=command)
        with self._condition:
            self._submit_unlocked(receipt, discard_pending=discard_pending)
            self._condition.notify()
        return receipt

    def submit_decision(self, decision: SafetyDecision) -> tuple[DispatchReceipt, ...]:
        """Queue an entire domain decision as one indivisible operation."""

        with self._condition:
            if not self._running:
                failed_receipts = tuple(
                    DispatchReceipt(command=command) for command in decision.commands
                )
                for receipt in failed_receipts:
                    receipt._complete(error="dispatcher is not running")
                    self._notify_receipt(receipt)
                return failed_receipts
            if len(decision.commands) > self._maximum_pending:
                raise BufferError("decision exceeds the bounded command queue")

            working = [] if decision.discard_pending_commands else list(self._pending)
            cancellations: list[tuple[_PendingCommand, str]] = []
            if decision.discard_pending_commands:
                cancellations.extend(
                    (item, "cancelled by a safe-state transition") for item in self._pending
                )

            planned_receipts: list[DispatchReceipt] = []
            for command in decision.commands:
                receipt = DispatchReceipt(command=command)
                planned_receipts.append(receipt)
                if command.is_motion or command.kind is CommandType.HEARTBEAT:
                    retained: list[_PendingCommand] = []
                    for item in working:
                        if item.command.kind is command.kind:
                            cancellations.append(
                                (item, f"superseded by newer {command.kind.value} command")
                            )
                        else:
                            retained.append(item)
                    working = retained
                while len(working) >= self._maximum_pending:
                    removable_index = next(
                        (
                            index
                            for index in range(len(working) - 1, -1, -1)
                            if working[index].priority >= int(command.priority)
                        ),
                        None,
                    )
                    if removable_index is None:
                        raise BufferError("command queue is full of higher-priority work")
                    removed = working.pop(removable_index)
                    cancellations.append((removed, "evicted by higher priority"))
                working.append(
                    _PendingCommand(
                        priority=int(command.priority),
                        order=next(self._order),
                        command=command,
                        receipt=receipt,
                        enqueued_at=time.monotonic(),
                    )
                )
                working.sort(key=lambda item: (item.priority, item.order))

            self._pending = working
            for item, reason in cancellations:
                item.receipt._complete(cancelled=True, error=reason)
                self._notify_receipt(item.receipt)
            self._condition.notify_all()
            return tuple(planned_receipts)

    def wait_for_idle(self, timeout_seconds: float) -> bool:
        deadline = time.monotonic() + timeout_seconds
        with self._condition:
            while self._pending or self._active:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._condition.wait(remaining)
            return True

    def stop(self, timeout_seconds: float = 2.0) -> None:
        with self._condition:
            self._running = False
            self._cancel_pending_unlocked("dispatcher stopped")
            self._condition.notify_all()
            thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout_seconds)
        with self._condition:
            if thread is not None and not thread.is_alive() and self._thread is thread:
                self._thread = None
            elif (
                thread is not None
                and thread.is_alive()
                and thread is not threading.current_thread()
            ):
                LOGGER.error(
                    "Actuator dispatcher did not stop within %.3f seconds", timeout_seconds
                )

    def _worker(self) -> None:
        try:
            self._transport.connect()
        except Exception as exc:  # noqa: BLE001 - adapters normalize arbitrary backend failures.
            with self._condition:
                self._start_error = f"could not start {self._transport.description}: {exc}"
                self._running = False
                self._started.set()
                self._condition.notify_all()
            return

        self._started.set()
        try:
            while True:
                with self._condition:
                    while self._running and not self._pending:
                        self._condition.wait(0.25)
                    if not self._running and not self._pending:
                        break
                    pending = self._pending.pop(0)
                    self._active = True
                self._dispatch(pending)
                with self._condition:
                    self._active = False
                    self._condition.notify_all()
        finally:
            try:
                self._transport.disconnect()
            except Exception:
                LOGGER.exception("Actuator transport cleanup failed")
            with self._condition:
                self._running = False
                self._active = False
                if self._thread is threading.current_thread():
                    self._thread = None
                self._condition.notify_all()

    def _dispatch(self, pending: _PendingCommand) -> None:
        receipt = pending.receipt
        sequence: int | None = None
        try:
            command_age = time.monotonic() - pending.enqueued_at
            if pending.command.requires_freshness and command_age > self._maximum_command_age:
                raise TimeoutError(
                    f"refusing stale {pending.command.kind.value} command aged {command_age:.3f}s"
                )
            sequence = self._next_sequence()
            receipt.sequence = sequence
            frame = self._codec.encode_command(pending.command, sequence).rstrip("\r\n")
            response = self._transport.transact(
                frame,
                self._ack_timeout if self._require_ack else 0.0,
            )
            if self._require_ack:
                if response is None:
                    raise ProtocolError(f"ACK timeout for sequence {sequence}")
                parsed = self._codec.parse_response(response, expected_sequence=sequence)
                if parsed.kind is ResponseKind.NACK:
                    raise ProtocolError(
                        f"firmware rejected sequence {sequence}: {parsed.fault_code}"
                    )
            receipt._complete(succeeded=True, response=response)
            LOGGER.debug("Dispatched %s as sequence %s", pending.command.kind.value, sequence)
        except Exception as exc:  # noqa: BLE001 - every transport failure completes the receipt.
            receipt._complete(error=str(exc))
            LOGGER.error(
                "Command %s sequence %s failed: %s",
                pending.command.kind.value,
                sequence,
                exc,
            )
        self._notify_receipt(receipt)

    def _submit_unlocked(
        self,
        receipt: DispatchReceipt,
        *,
        discard_pending: bool,
    ) -> None:
        command = receipt.command
        if not self._running:
            receipt._complete(error="dispatcher is not running")
            self._notify_receipt(receipt)
            return
        if discard_pending:
            self._cancel_pending_unlocked("superseded by a safety decision")
        if command.is_motion or command.kind is CommandType.HEARTBEAT:
            self._coalesce_kind_unlocked(command.kind)
        self._make_room_unlocked(command.priority)
        self._pending.append(
            _PendingCommand(
                priority=int(command.priority),
                order=next(self._order),
                command=command,
                receipt=receipt,
                enqueued_at=time.monotonic(),
            )
        )
        self._pending.sort(key=lambda item: (item.priority, item.order))

    def _next_sequence(self) -> int:
        with self._condition:
            if self._sequence > MAX_SEQUENCE:
                raise ProtocolError(
                    "protocol sequence space exhausted; reconnect before continuing"
                )
            sequence = self._sequence
            self._sequence += 1
            return sequence

    def _coalesce_kind_unlocked(self, kind: CommandType) -> None:
        retained: list[_PendingCommand] = []
        for item in self._pending:
            if item.command.kind is kind:
                item.receipt._complete(
                    cancelled=True,
                    error=f"superseded by newer {kind.value} command",
                )
                self._notify_receipt(item.receipt)
            else:
                retained.append(item)
        self._pending = retained

    def _make_room_unlocked(self, incoming_priority: CommandPriority) -> None:
        while len(self._pending) >= self._maximum_pending:
            removable_index = next(
                (
                    index
                    for index in range(len(self._pending) - 1, -1, -1)
                    if self._pending[index].priority >= int(incoming_priority)
                ),
                None,
            )
            if removable_index is None:
                raise BufferError("command queue is full of higher-priority work")
            removed = self._pending.pop(removable_index)
            removed.receipt._complete(cancelled=True, error="evicted by higher priority")
            self._notify_receipt(removed.receipt)

    def _cancel_pending_unlocked(self, reason: str) -> None:
        cancelled, self._pending = self._pending, []
        for item in cancelled:
            item.receipt._complete(cancelled=True, error=reason)
            self._notify_receipt(item.receipt)

    def _notify_receipt(self, receipt: DispatchReceipt) -> None:
        if self._on_receipt is None:
            return
        try:
            self._on_receipt(receipt)
        except Exception:
            LOGGER.exception("Command receipt callback failed")
