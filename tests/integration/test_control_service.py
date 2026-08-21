from __future__ import annotations

import time
import unittest
from collections.abc import Callable

from car_interface.adapters.simulated import (
    SimulatedControllerSource,
    SimulatedLidarSource,
    SimulatedVehicleTransport,
)
from car_interface.config import AppConfig
from car_interface.services.control import ControlService
from car_interface.services.models import ServiceSnapshot


class ControlServiceIntegrationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.vehicle = SimulatedVehicleTransport()
        self.lidar = SimulatedLidarSource(obstacle_distance_cm=180)
        self.controller = SimulatedControllerSource()
        self.service = ControlService(
            AppConfig(),
            vehicle=self.vehicle,
            lidar=self.lidar,
            controller=self.controller,
        )

    def tearDown(self) -> None:
        self.service.shutdown()

    def wait_until(
        self,
        predicate: Callable[[ServiceSnapshot], bool],
        timeout: float = 2.0,
    ) -> ServiceSnapshot:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            snapshot = self.service.snapshot()
            if predicate(snapshot):
                return snapshot
            time.sleep(0.01)
        return self.fail(f"condition not reached; final snapshot={self.service.snapshot()!r}")

    def test_connect_arm_drive_and_operator_estop(self) -> None:
        self.service.connect()
        self.wait_until(lambda state: state.phase == "safe_connected")
        self.service.arm()
        self.wait_until(lambda state: state.phase == "armed")

        self.service.set_speed(20)
        self.wait_until(lambda state: state.phase == "driving")
        self.assertTrue(self._wait_value(lambda: self.vehicle.speed_percent == 20))

        self.service.emergency_stop("integration test")
        stopped = self.wait_until(lambda state: state.estop_active)
        self.assertEqual(stopped.speed_percent, 0)
        self.assertTrue(self._wait_value(lambda: self.vehicle.estopped))
        self.assertTrue(self.vehicle.braked)

    def test_controller_loss_requests_safe_fault(self) -> None:
        self.service.connect()
        self.wait_until(lambda state: state.phase == "safe_connected")
        self.service.arm()
        self.service.set_speed(15)
        self.wait_until(lambda state: state.phase == "driving")

        self.controller.disconnect()
        fault = self.wait_until(lambda state: state.fault is not None)

        self.assertIn("controller", fault.fault.lower())
        self.assertEqual(fault.speed_percent, 0)
        self.assertTrue(self._wait_value(lambda: self.vehicle.estopped))

    def test_lidar_obstacle_triggers_estop(self) -> None:
        self.service.connect()
        self.wait_until(lambda state: state.phase == "safe_connected")
        self.service.arm()
        self.service.set_speed(10)
        self.wait_until(lambda state: state.phase == "driving")

        self.lidar.set_obstacle_distance(25)
        stopped = self.wait_until(lambda state: state.estop_active)

        self.assertLess(stopped.closest_obstacle_cm or 999, 50)
        self.assertTrue(self._wait_value(lambda: self.vehicle.braked))

    def test_dispatch_failure_loses_actuator_and_stops_supervision(self) -> None:
        self.service.connect()
        self.wait_until(lambda state: state.phase == "safe_connected")
        self.service.arm()
        self.vehicle.fail_next_transaction = True

        self.service.set_speed(10)
        disconnected = self.wait_until(lambda state: not state.vehicle_connected)

        self.assertEqual(disconnected.phase, "disconnected")
        self.assertIsNotNone(disconnected.fault)

    @staticmethod
    def _wait_value(predicate: Callable[[], bool], timeout: float = 1.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.01)
        return predicate()


if __name__ == "__main__":
    unittest.main()
