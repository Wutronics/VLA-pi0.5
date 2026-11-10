from __future__ import annotations

import unittest

import numpy as np

from src.simulation import D3RightArmSim


class D3RightArmSimTest(unittest.TestCase):
    def setUp(self) -> None:
        self.sim = D3RightArmSim(render=False, realtime=False)

    def tearDown(self) -> None:
        self.sim.close()

    def test_model_contract(self) -> None:
        self.assertEqual(len(self.sim.state().joint_position_rad), 7)
        self.assertEqual(len(self.sim.right_joint_limits_rad), 7)
        self.assertAlmostEqual(self.sim.timestep_s, 0.001)

    def test_invalid_joint_action_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            self.sim.move_joint([0.0] * 6)
        with self.assertRaises(ValueError):
            self.sim.move_joint([100.0] * 7)

    def test_right_joint_move_keeps_left_frozen(self) -> None:
        target = np.array([0.15, -0.30, 0.10, 0.40, 0.0, 0.10, 0.0])
        left_before = np.asarray(self.sim.left_joint_position_rad)
        state = self.sim.move_joint(target, duration_s=1.2, settle_s=0.2)
        np.testing.assert_allclose(state.joint_position_rad, target, atol=0.08)
        np.testing.assert_allclose(
            self.sim.left_joint_position_rad, left_before, atol=1e-12
        )

    def test_forward_inverse_kinematics_round_trip(self) -> None:
        target = np.array([0.15, -0.30, 0.10, 0.40, 0.0, 0.10, 0.0])
        pose = np.asarray(self.sim.forward_kinematics(target))
        solution = self.sim.inverse_kinematics(
            pose, reference_joint_rad=target
        )
        reconstructed = np.asarray(self.sim.forward_kinematics(solution))
        np.testing.assert_allclose(reconstructed[:3], pose[:3], atol=0.03)


if __name__ == "__main__":
    unittest.main()
