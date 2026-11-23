from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from src.robots.d3 import (
    D3RightArmSafetyAdapter,
    RightArmSafetyConfig,
    SafetyViolation,
)


class _FakeD3Client:
    """In-memory SDK double; no test in this module contacts a controller."""

    def __init__(self) -> None:
        self.left_position = [0.0] * 7
        self.right_position = [0.0] * 7
        self.calls: list[tuple[str, object]] = []

    @staticmethod
    def _config() -> dict[str, object]:
        return {"dof": 7, "limit_lower": [-1.0] * 7, "limit_upper": [1.0] * 7}

    def get_left_config(self) -> dict[str, object]:
        self.calls.append(("get_left_config", None))
        return self._config()

    def get_right_config(self) -> dict[str, object]:
        self.calls.append(("get_right_config", None))
        return self._config()

    def get_left_status(self) -> dict[str, object]:
        self.calls.append(("get_left_status", None))
        return {"arm_is_connected": True}

    def get_right_status(self) -> dict[str, object]:
        self.calls.append(("get_right_status", None))
        return {"servo": 1, "fsm_state": "POSITION", "motion_state": 1}

    def get_left_joint_pos(self) -> list[float]:
        self.calls.append(("get_left_joint_pos", None))
        return self.left_position.copy()

    def get_right_joint_pos(self) -> list[float]:
        self.calls.append(("get_right_joint_pos", None))
        return self.right_position.copy()

    def get_right_joint_vel(self) -> list[float]:
        self.calls.append(("get_right_joint_vel", None))
        return [0.0] * 7

    def move_right_joint(self, targets: list[float], desire_time: float, is_sync: bool) -> int:
        self.calls.append(("move_right_joint", (targets, desire_time, is_sync)))
        self.right_position = targets.copy()
        return 1

    def track_right_joint(self, targets: list[float], eeff_pos: float = -1.0) -> int:
        self.calls.append(("track_right_joint", (targets, eeff_pos)))
        return 1

    def task_stop(self) -> int:
        self.calls.append(("task_stop", None))
        return 1

    def disconnect(self) -> int:
        self.calls.append(("disconnect", None))
        return 1


class D3RightArmSafetyAdapterTest(unittest.TestCase):
    def setUp(self) -> None:
        self.client = _FakeD3Client()
        self.adapter = D3RightArmSafetyAdapter(
            self.client,
            RightArmSafetyConfig(max_joint_step_rad=0.05, minimum_move_duration_s=3.0),
        )

    def test_constructor_has_no_sdk_side_effect(self) -> None:
        self.assertEqual(self.client.calls, [])

    def test_move_requires_explicit_preflight_and_arm(self) -> None:
        with self.assertRaisesRegex(SafetyViolation, "motion is not armed"):
            self.adapter.move_joint([0.01] + [0.0] * 6, duration_s=3.0)
        self.assertFalse(any(name == "move_right_joint" for name, _ in self.client.calls))

    def test_move_is_right_only_and_writes_audit_log(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            log_path = Path(directory) / "right-arm.jsonl"
            adapter = D3RightArmSafetyAdapter(
                self.client,
                RightArmSafetyConfig(
                    max_joint_step_rad=0.05,
                    minimum_move_duration_s=3.0,
                    log_path=log_path,
                ),
            )
            adapter.arm_motion()
            after = adapter.move_joint([0.02] + [0.0] * 6, duration_s=3.0)

            self.assertEqual(after.right_joint_position_rad[0], 0.02)
            move_calls = [payload for name, payload in self.client.calls if name == "move_right_joint"]
            self.assertEqual(move_calls, [([0.02] + [0.0] * 6, 3.0, True)])
            self.assertFalse(any(name.startswith("move_left") for name, _ in self.client.calls))
            self.assertIn("right_move_joint_completed", log_path.read_text(encoding="utf-8"))

    def test_rejects_target_above_step_limit_without_sending_motion(self) -> None:
        self.adapter.arm_motion()
        with self.assertRaisesRegex(SafetyViolation, "step limit"):
            self.adapter.move_joint([0.06] + [0.0] * 6, duration_s=3.0)
        self.assertFalse(any(name == "move_right_joint" for name, _ in self.client.calls))

    def test_tracking_is_disabled_by_default(self) -> None:
        self.adapter.arm_motion()
        with self.assertRaisesRegex(SafetyViolation, "tracking is disabled"):
            self.adapter.track_joint([0.01] + [0.0] * 6)
        self.assertFalse(any(name == "track_right_joint" for name, _ in self.client.calls))

    def test_close_only_disconnects_when_tracking_is_not_active(self) -> None:
        self.adapter.close()
        self.assertEqual(self.client.calls, [("disconnect", None)])


if __name__ == "__main__":
    unittest.main()
