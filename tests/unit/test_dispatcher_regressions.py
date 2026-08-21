from __future__ import annotations

import threading
import time

import pytest

from car_interface.domain import (
    MAX_SEQUENCE,
    CommandType,
    ControlCommand,
    ProtocolCodec,
    SafetyDecision,
    SafetyStateMachine,
)
from car_interface.services.dispatcher import CommandDispatcher, DispatchReceipt


class AckTransport:
    def __init__(self, *, gate_first_transaction: bool = False) -> None:
        self.connected = False
        self.frames: list[str] = []
        self.first_transaction_entered = threading.Event()
        self.release_first_transaction = threading.Event()
        self._gate_first_transaction = gate_first_transaction
        self._transactions = 0
        self._lock = threading.Lock()
        self._codec = ProtocolCodec()

    @property
    def is_connected(self) -> bool:
        return self.connected

    @property
    def description(self) -> str:
        return "test:ack-transport"

    def connect(self) -> None:
        self.connected = True

    def transact(self, frame: str, timeout_seconds: float) -> str:
        del timeout_seconds
        with self._lock:
            transaction = self._transactions
            self._transactions += 1
            self.frames.append(frame)
        if self._gate_first_transaction and transaction == 0:
            self.first_transaction_entered.set()
            if not self.release_first_transaction.wait(2.0):
                raise TimeoutError("test did not release the first transaction")
        decoded = self._codec.parse_command(frame)
        return self._codec.encode_ack(decoded.sequence)

    def disconnect(self) -> None:
        self.connected = False


def _wait_finished(receipt: DispatchReceipt, timeout: float = 1.0) -> None:
    deadline = time.monotonic() + timeout
    while not receipt.done and time.monotonic() < deadline:
        time.sleep(0.005)
    assert receipt.done


def _decoded_kinds(transport: AckTransport) -> list[CommandType]:
    codec = ProtocolCodec()
    return [codec.parse_command(frame).command.kind for frame in transport.frames]


def test_safe_decision_atomically_discards_motion_and_dispatches_estop_first() -> None:
    transport = AckTransport(gate_first_transaction=True)
    dispatcher = CommandDispatcher(transport)
    dispatcher.start()
    blocker = dispatcher.submit(ControlCommand.set_speed(0))
    assert transport.first_transaction_entered.wait(1.0)

    speed = dispatcher.submit(ControlCommand.set_speed(40))
    steering = dispatcher.submit(ControlCommand.set_steering(25))
    machine = SafetyStateMachine()
    machine.connect(0.0)
    safe_receipts = dispatcher.submit_decision(machine.emergency_stop("dispatcher regression", 0.1))

    assert speed.cancelled
    assert steering.cancelled
    assert "safe-state transition" in (speed.error or "")
    transport.release_first_transaction.set()
    try:
        assert blocker.wait(1.0)
        assert all(receipt.wait(1.0) for receipt in safe_receipts)
        assert dispatcher.wait_for_idle(1.0)
        assert _decoded_kinds(transport) == [
            CommandType.SET_SPEED,
            CommandType.EMERGENCY_STOP,
            CommandType.SET_SPEED,
            CommandType.SET_BRAKE,
            CommandType.SET_ARMED,
        ]
    finally:
        dispatcher.stop()


def test_queued_heartbeats_are_coalesced_to_the_latest_sample() -> None:
    transport = AckTransport(gate_first_transaction=True)
    dispatcher = CommandDispatcher(transport)
    dispatcher.start()
    blocker = dispatcher.submit(ControlCommand.set_speed(0))
    assert transport.first_transaction_entered.wait(1.0)

    first = dispatcher.submit(ControlCommand.heartbeat())
    second = dispatcher.submit(ControlCommand.heartbeat())
    latest = dispatcher.submit(ControlCommand.heartbeat())

    assert first.cancelled
    assert second.cancelled
    assert not latest.done
    transport.release_first_transaction.set()
    try:
        assert blocker.wait(1.0)
        assert latest.wait(1.0)
        assert _decoded_kinds(transport).count(CommandType.HEARTBEAT) == 1
    finally:
        dispatcher.stop()


def test_sequence_exhaustion_fails_receipt_without_wrapping() -> None:
    transport = AckTransport()
    dispatcher = CommandDispatcher(transport)
    dispatcher.start()
    try:
        dispatcher._sequence = MAX_SEQUENCE
        final_valid = dispatcher.submit(ControlCommand.set_speed(0))
        assert final_valid.wait(1.0)
        assert final_valid.sequence == MAX_SEQUENCE

        exhausted = dispatcher.submit(ControlCommand.set_brake(True))
        _wait_finished(exhausted)
        assert not exhausted.succeeded
        assert exhausted.sequence is None
        assert "sequence space exhausted" in (exhausted.error or "")
    finally:
        dispatcher.stop()


def test_failed_batch_planning_leaves_existing_queue_and_receipts_untouched() -> None:
    transport = AckTransport(gate_first_transaction=True)
    dispatcher = CommandDispatcher(transport, maximum_pending=8)
    dispatcher.start()
    blocker = dispatcher.submit(ControlCommand.set_speed(0))
    assert transport.first_transaction_entered.wait(1.0)
    queued = [dispatcher.submit(ControlCommand.emergency_stop()) for _ in range(8)]
    machine = SafetyStateMachine()
    state = machine.snapshot
    decision = SafetyDecision(
        action="atomic batch regression",
        previous=state,
        current=state,
        commands=(ControlCommand.emergency_stop(), ControlCommand.heartbeat()),
    )

    with pytest.raises(BufferError, match="higher-priority work"):
        dispatcher.submit_decision(decision)

    assert all(not receipt.done for receipt in queued)
    transport.release_first_transaction.set()
    try:
        assert blocker.wait(1.0)
        assert all(receipt.wait(1.0) for receipt in queued)
        assert dispatcher.wait_for_idle(1.0)
        assert _decoded_kinds(transport) == [
            CommandType.SET_SPEED,
            *([CommandType.EMERGENCY_STOP] * 8),
        ]
    finally:
        dispatcher.stop()
