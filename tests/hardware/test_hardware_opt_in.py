from __future__ import annotations

import os

import pytest

pytestmark = pytest.mark.hardware


def test_hardware_suite_is_inert_without_an_explicit_configured_rig() -> None:
    """This sentinel intentionally opens no device and issues no command."""

    assert os.environ["CAR_INTERFACE_HARDWARE_ACKNOWLEDGED"] == "1"
    pytest.skip("configure a documented test rig before adding device-specific checks")
