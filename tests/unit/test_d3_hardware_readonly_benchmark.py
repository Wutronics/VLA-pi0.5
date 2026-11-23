from __future__ import annotations

import unittest

from scripts.evaluate.d3_hardware_readonly_benchmark import collect_snapshot


class _ReadOnlyRobot:
    def __init__(self) -> None:
        self.calls: list[str] = []

    def _record(self, name: str, value):
        self.calls.append(name)
        return value

    def get_version(self):
        return self._record("get_version", "test-sdk")

    def get_left_config(self):
        return self._record("get_left_config", {"dof": 7})

    def get_right_config(self):
        return self._record("get_right_config", {"dof": 7})

    def get_left_status(self):
        return self._record("get_left_status", {"arm_is_connected": True})

    def get_right_status(self):
        return self._record("get_right_status", {"arm_is_connected": True})

    def get_left_joint_pos(self):
        return self._record("get_left_joint_pos", [0.0] * 7)

    def get_right_joint_pos(self):
        return self._record("get_right_joint_pos", [0.0] * 7)

    def get_left_joint_vel(self):
        return self._record("get_left_joint_vel", [0.0] * 7)

    def get_right_joint_vel(self):
        return self._record("get_right_joint_vel", [0.0] * 7)

    def get_left_joint_tau(self):
        return self._record("get_left_joint_tau", [0.0] * 7)

    def get_right_joint_tau(self):
        return self._record("get_right_joint_tau", [0.0] * 7)

    def get_left_cart_pose(self):
        return self._record("get_left_cart_pose", [0.0] * 7)

    def get_right_cart_pose(self):
        return self._record("get_right_cart_pose", [0.0] * 7)


class D3HardwareReadonlyBenchmarkTest(unittest.TestCase):
    def test_snapshot_has_right_arm_feedback_and_uses_only_getters(self) -> None:
        robot = _ReadOnlyRobot()
        snapshot = collect_snapshot(robot)

        self.assertEqual(snapshot["sdk_version"], "test-sdk")
        self.assertEqual(len(snapshot["right"]["joint_position_rad"]), 7)
        self.assertEqual(len(snapshot["right"]["flange_pose"]), 7)
        self.assertTrue(robot.calls)
        self.assertTrue(all(call.startswith("get_") for call in robot.calls))

    def test_snapshot_rejects_wrong_dof(self) -> None:
        robot = _ReadOnlyRobot()
        robot.get_right_config = lambda: {"dof": 6}
        with self.assertRaisesRegex(ValueError, "right arm reports 6 DOF"):
            collect_snapshot(robot)


if __name__ == "__main__":
    unittest.main()
