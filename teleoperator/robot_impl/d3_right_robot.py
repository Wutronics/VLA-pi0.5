"""CVTE D3 right-arm-only adapter for the exoskeleton recorder.

The old recorder emits absolute targets for two arms.  This controller ignores
the left target and maps only the right exoskeleton *delta* around a captured
reference onto the current D3 right-arm pose.  Capturing the reference never
moves the robot.
"""

from __future__ import annotations

import math
import os
from pathlib import Path
import sys
from typing import Iterable

import numpy as np

try:
    from ..robot_base import BaseRobotController
except ImportError:  # direct execution through teleoperator/main.py
    from robot_base import BaseRobotController

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from src.robots.d3 import D3RightArmSafetyAdapter, RightArmSafetyConfig, SafetyViolation


def _vector7(values: Iterable[float], label: str) -> np.ndarray:
    vector = np.asarray(list(values), dtype=np.float64)
    if vector.shape != (7,):
        raise ValueError(f"{label} must contain exactly 7 values")
    if not np.isfinite(vector).all():
        raise ValueError(f"{label} contains NaN or infinity")
    return vector


class D3RightArmController(BaseRobotController):
    """Right-arm-only controller with dry-run as the default mode."""

    requires_home = False
    alignment_moves_robot = False
    EXECUTE_CONFIRMATION = "I_UNDERSTAND_D3_RIGHT_ARM_ONLY"

    def __init__(self, robot_cfg: dict):
        self.cfg = dict(robot_cfg)
        self.dry_run = bool(self.cfg.get("dry_run", True))
        self._max_step_rad = math.radians(float(self.cfg.get("max_joint_step_deg", 1.0)))
        if not math.isfinite(self._max_step_rad) or self._max_step_rad <= 0:
            raise ValueError("max_joint_step_deg must be finite and positive")
        mapping = self.cfg.get("relative_mapping", {})
        self._indices = np.asarray(mapping.get("exo_indices", list(range(7))), dtype=int)
        self._sign = _vector7(mapping.get("sign", [1.0] * 7), "relative_mapping.sign")
        self._gain = _vector7(mapping.get("gain", [1.0] * 7), "relative_mapping.gain")
        if self._indices.shape != (7,) or sorted(self._indices.tolist()) != list(range(7)):
            raise ValueError("relative_mapping.exo_indices must be a permutation of 0..6")
        if (self._gain <= 0).any():
            raise ValueError("relative_mapping.gain values must be positive")
        if not np.isin(self._sign, [-1.0, 1.0]).all():
            raise ValueError("relative_mapping.sign values must be +1 or -1")

        configured_limits = self.cfg.get("joint_limits_rad")
        self._joint_limits: np.ndarray | None = None
        if configured_limits is not None:
            limits = np.asarray(configured_limits, dtype=np.float64)
            if limits.shape != (7, 2) or not np.isfinite(limits).all():
                raise ValueError("joint_limits_rad must contain seven finite [min, max] pairs")
            if (limits[:, 0] >= limits[:, 1]).any():
                raise ValueError("joint_limits_rad contains an invalid interval")
            self._joint_limits = limits

        self._adapter: D3RightArmSafetyAdapter | None = None
        self._exo_reference: np.ndarray | None = None
        self._robot_reference: np.ndarray | None = None
        self._latest_command = np.zeros(7, dtype=np.float64)
        self._dry_state = np.zeros(7, dtype=np.float64)

    def _selected_exo(self, right_target: Iterable[float]) -> np.ndarray:
        return _vector7(right_target, "right exoskeleton target")[self._indices]

    def open(self, speed=10) -> bool:
        del speed  # D3 speed is configured and verified on the pendant.
        if self.dry_run:
            print("[D3] DRY RUN：不连接控制器、不发送运动命令")
            return True

        if not self.cfg.get("mapping_verified", False):
            raise SafetyViolation("D3 joint mapping has not been marked as verified")
        if not self.cfg.get("tracking_enabled", False):
            raise SafetyViolation("D3 tracking is disabled in config")
        if not self.cfg.get("global_task_stop_verified", False):
            raise SafetyViolation("D3 task_stop path has not been marked as verified")
        if os.environ.get("D3_TELEOP_EXECUTE") != self.EXECUTE_CONFIRMATION:
            raise SafetyViolation(
                "set D3_TELEOP_EXECUTE=I_UNDERSTAND_D3_RIGHT_ARM_ONLY for live teleoperation"
            )
        address_env = self.cfg.get("address_env", "D3_CONTROLLER_ADDRESS")
        address = os.environ.get(address_env)
        if not address:
            raise SafetyViolation(f"controller address environment variable {address_env} is unset")

        safety = RightArmSafetyConfig(
            max_joint_step_rad=self._max_step_rad,
            minimum_move_duration_s=float(self.cfg.get("minimum_move_duration_s", 3.0)),
            tracking_enabled=True,
            stale_command_timeout_s=float(self.cfg.get("stale_command_timeout_ms", 250)) / 1000.0,
            global_task_stop_verified=True,
            log_path=Path(self.cfg.get("command_log_path", "artifacts/evaluation/d3-teleop.jsonl")),
        )
        self._adapter = D3RightArmSafetyAdapter.from_carm(
            address,
            port=int(self.cfg.get("port", 8090)),
            timeout=float(self.cfg.get("timeout_s", 1.0)),
            config=safety,
        )
        snapshot = self._adapter.arm_motion()
        self._latest_command = np.asarray(snapshot.right_joint_position_rad, dtype=np.float64)
        return True

    def close(self):
        if self._adapter is not None:
            self._adapter.close()
            self._adapter = None

    def align_for_teleop(self, left_target, right_target, duration, frequency):
        del left_target, duration, frequency
        self._exo_reference = self._selected_exo(right_target)
        if self._adapter is None:
            self._robot_reference = self._dry_state.copy()
        else:
            snapshot = self._adapter.arm_motion()
            self._robot_reference = np.asarray(snapshot.right_joint_position_rad, dtype=np.float64)
        self._latest_command = self._robot_reference.copy()
        print("[D3] 已捕获相对映射参考姿态；机器人未运动")

    def _map_target(self, right_target: Iterable[float]) -> np.ndarray:
        if self._exo_reference is None or self._robot_reference is None:
            raise SafetyViolation("capture a D3/exoskeleton reference before teleoperation")
        exo = self._selected_exo(right_target)
        return self._robot_reference + (exo - self._exo_reference) * self._sign * self._gain

    def set_joint_target(self, left_target, right_target, duration, frequency):
        del left_target, duration, frequency
        target = self._map_target(right_target)
        if not np.isfinite(target).all():
            raise SafetyViolation("mapped D3 target contains NaN or infinity")
        if self._joint_limits is not None:
            if ((target < self._joint_limits[:, 0]) | (target > self._joint_limits[:, 1])).any():
                raise SafetyViolation("mapped D3 target is outside configured joint limits")
        if np.max(np.abs(target - self._latest_command)) > self._max_step_rad:
            raise SafetyViolation("mapped D3 target exceeds the per-command joint-step limit")
        if self._adapter is not None:
            self._adapter.track_joint(target.tolist())
        self._latest_command = target.copy()

    def get_joint_state(self) -> np.ndarray:
        if self._adapter is None:
            return self._dry_state.astype(np.float32, copy=True)
        snapshot = self._adapter.read_snapshot()
        return np.asarray(snapshot.right_joint_position_rad, dtype=np.float32)

    def action_for_dataset(self, exo_action: np.ndarray) -> np.ndarray:
        del exo_action
        return self._latest_command.astype(np.float32, copy=True)

    def stop_motion(self, reason: str = "operator_requested"):
        if self._adapter is not None:
            self._adapter.stop_motion(reason)
        self._exo_reference = None
        self._robot_reference = None

    def tick(self):
        if self._adapter is not None:
            self._adapter.enforce_watchdog()

    def set_speed(self, v, w):
        del v, w  # D3 fixed-base platform has no mobile-base command.
