from __future__ import annotations

import os

import pytest


@pytest.fixture(autouse=True)
def require_explicit_hardware_acknowledgement() -> None:
    """Keep every hardware test inert unless the operator opts in explicitly."""

    if os.environ.get("CAR_INTERFACE_HARDWARE_ACKNOWLEDGED") != "1":
        pytest.skip("set CAR_INTERFACE_HARDWARE_ACKNOWLEDGED=1 to opt in")
