from __future__ import annotations

import unittest

import numpy as np

from teleoperator.robot_impl.d3_right_robot import D3RightArmController


def _config() -> dict:
    return {
        "dry_run": True,
        "max_joint_step_deg": 180.0,
        "relative_mapping": {
            "exo_indices": [1, 0, 2, 3, 4, 5, 6],
            "sign": [1, -1, 1, 1, 1, 1, 1],
            "gain": [0.5] * 7,
        },
    }


class D3ExoskeletonMappingTest(unittest.TestCase):
    def test_dry_run_never_constructs_hardware_and_maps_only_right_delta(self) -> None:
        controller = D3RightArmController(_config())
        self.assertTrue(controller.open())
        controller.align_for_teleop(
            left_target=[50.0] * 7,
            right_target=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0],
            duration=2.0,
            frequency=100,
        )
        controller.set_joint_target(
            left_target=[-50.0] * 7,
            right_target=[1.2, 2.4, 3.6, 4.8, 6.0, 7.2, 8.4],
            duration=0.1,
            frequency=10,
        )

        action = controller.action_for_dataset(np.zeros(14, dtype=np.float32))
        expected = np.array([0.2, -0.1, 0.3, 0.4, 0.5, 0.6, 0.7], dtype=np.float32)
        np.testing.assert_allclose(action, expected, atol=1e-6)
        np.testing.assert_array_equal(controller.get_joint_state(), np.zeros(7, dtype=np.float32))

    def test_mapping_requires_reference_capture(self) -> None:
        controller = D3RightArmController(_config())
        controller.open()
        with self.assertRaisesRegex(RuntimeError, "capture a D3/exoskeleton reference"):
            controller.set_joint_target([0.0] * 7, [0.0] * 7, 0.1, 10)

    def test_invalid_joint_permutation_is_rejected(self) -> None:
        config = _config()
        config["relative_mapping"]["exo_indices"] = [0] * 7
        with self.assertRaisesRegex(ValueError, "permutation"):
            D3RightArmController(config)

    def test_dry_run_rate_limits_per_command_jump(self) -> None:
        config = _config()
        config["max_joint_step_deg"] = 1.0
        controller = D3RightArmController(config)
        controller.open()
        controller.align_for_teleop([0.0] * 7, [0.0] * 7, 2.0, 10)
        controller.set_joint_target([0.0] * 7, [0.1] + [0.0] * 6, 0.1, 10)

        action = controller.action_for_dataset(np.zeros(14, dtype=np.float32))
        self.assertAlmostEqual(float(np.max(np.abs(action))), np.radians(1.0), places=6)


if __name__ == "__main__":
    unittest.main()
