from __future__ import annotations

import unittest

from car_interface.adapters.base import ScanPoint
from car_interface.services.lidar_analysis import assess_scan


class LidarAssessmentTests(unittest.TestCase):
    def test_closest_point_is_limited_to_projected_vehicle_path(self) -> None:
        assessment = assess_scan(
            (
                ScanPoint(angle_degrees=0, distance_cm=80),
                ScanPoint(angle_degrees=45, distance_cm=40),
                ScanPoint(angle_degrees=2, distance_cm=55),
            ),
            captured_at=10,
            now=10.1,
            vehicle_width_cm=42,
            stale_after_seconds=0.5,
        )

        self.assertEqual(assessment.closest_in_path_cm, 55)
        self.assertEqual(assessment.closest_angle_degrees, 2)
        self.assertFalse(assessment.stale)

    def test_old_scan_is_explicitly_stale(self) -> None:
        assessment = assess_scan(
            (ScanPoint(angle_degrees=0, distance_cm=20),),
            captured_at=1,
            now=2,
            vehicle_width_cm=42,
            stale_after_seconds=0.5,
        )

        self.assertTrue(assessment.stale)

    def test_invalid_measurements_are_ignored(self) -> None:
        assessment = assess_scan(
            (
                ScanPoint(angle_degrees=0, distance_cm=float("inf")),
                ScanPoint(angle_degrees=0, distance_cm=-1),
            ),
            captured_at=1,
            now=1,
            vehicle_width_cm=42,
            stale_after_seconds=0.5,
        )

        self.assertIsNone(assessment.closest_in_path_cm)


if __name__ == "__main__":
    unittest.main()
