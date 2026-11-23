from __future__ import annotations

import unittest

from scripts.evaluate.d3_right_joint_step_benchmark import plan_joint_step


def _snapshot() -> dict:
    return {
        "left": {
            "config": {"limit_lower": [-3.0] * 7, "limit_upper": [3.0] * 7},
            "joint_position_rad": [0.0] * 7,
        },
        "right": {
            "config": {"limit_lower": [-3.0] * 7, "limit_upper": [3.0] * 7},
            "joint_position_rad": [0.0] * 7,
            "status": {"servo": 1, "fsm_state": "POSITION", "motion_state": 1},
        },
    }


class D3RightJointStepBenchmarkTest(unittest.TestCase):
    def test_changes_only_requested_joint(self) -> None:
        target = plan_joint_step(_snapshot(), joint_index_1based=1, delta_deg=2.0)
        self.assertAlmostEqual(target[0], 0.034906585, places=7)
        self.assertEqual(target[1:], [0.0] * 6)

    def test_rejects_step_larger_than_two_degrees(self) -> None:
        with self.assertRaisesRegex(ValueError, "at most 2.0 deg"):
            plan_joint_step(_snapshot(), joint_index_1based=1, delta_deg=2.1)

    def test_rejects_disabled_right_servo(self) -> None:
        snapshot = _snapshot()
        snapshot["right"]["status"]["servo"] = 0
        with self.assertRaisesRegex(ValueError, "servo is not enabled"):
            plan_joint_step(snapshot, joint_index_1based=1, delta_deg=2.0)


if __name__ == "__main__":
    unittest.main()
