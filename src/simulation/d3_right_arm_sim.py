"""Right-arm-only adapter for CVTE's official D3 MuJoCo simulation.

The upstream simulator exposes both arms. This adapter intentionally exposes
only right-arm commands and kinematically freezes the left arm at its initial
pose. It is a learning and software-contract tool, not a real-hardware safety
controller.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys
import time
from typing import Iterable

import mujoco
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[2]
VENDOR_ROOT = PROJECT_ROOT / "third_party" / "carm-mujoco"
DEFAULT_MODEL_PATH = VENDOR_ROOT / "models" / "carm_d3_mjcf" / "carm_d3.xml"

if str(VENDOR_ROOT) not in sys.path:
    sys.path.insert(0, str(VENDOR_ROOT))

try:
    from carm_mujoco import CArmDualBot
except ImportError as exc:  # pragma: no cover - exercised by setup failures
    raise RuntimeError(
        "Missing CVTE carm-mujoco dependency. Run "
        "bash scripts/simulation/setup_simulation.sh first."
    ) from exc


@dataclass(frozen=True)
class RightArmState:
    """A read-only snapshot using the same units as the vendor API."""

    joint_position_rad: tuple[float, ...]
    joint_velocity_rad_s: tuple[float, ...]
    joint_torque_nm: tuple[float, ...]
    flange_pose_xyz_quat: tuple[float, ...]
    gripper_width_m: float
    simulation_time_s: float


class D3RightArmSim:
    """Constrained interface for practicing this project's right-arm scope."""

    DOF = 7

    def __init__(
        self,
        model_path: str | Path = DEFAULT_MODEL_PATH,
        *,
        render: bool = False,
        realtime: bool | None = None,
    ) -> None:
        model_path = Path(model_path).resolve()
        if not model_path.is_file():
            raise FileNotFoundError(
                f"D3 model not found at {model_path}. Run the simulation setup first."
            )

        self._robot = CArmDualBot(str(model_path), render=render)
        self._render = render
        self._realtime = render if realtime is None else realtime
        self._left_initial = np.asarray(
            self._robot.get_left_joint_pos(), dtype=float
        )
        self._right_joint_ids = [
            mujoco.mj_name2id(
                self._robot.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                f"joint{index}-RIGHT",
            )
            for index in range(1, self.DOF + 1)
        ]
        if any(joint_id < 0 for joint_id in self._right_joint_ids):
            self.close()
            raise RuntimeError("Official D3 model is missing one or more right-arm joints.")

        self._right_limits = self._robot.model.jnt_range[
            self._right_joint_ids
        ].copy()
        if self._robot.set_control_mode(1) != 1:
            self.close()
            raise RuntimeError("Failed to enter position control mode.")
        self._freeze_left()

    @property
    def timestep_s(self) -> float:
        return float(self._robot.model.opt.timestep)

    @property
    def right_joint_limits_rad(self) -> tuple[tuple[float, float], ...]:
        return tuple((float(low), float(high)) for low, high in self._right_limits)

    @property
    def left_joint_position_rad(self) -> tuple[float, ...]:
        """Read-only diagnostic used to verify that the left arm stays frozen."""
        return tuple(float(value) for value in self._robot.get_left_joint_pos())

    def _freeze_left(self) -> None:
        left_slice = self._robot.LEFT_ARM_SLICE
        self._robot.data.qpos[left_slice] = self._left_initial
        self._robot.data.qvel[left_slice] = 0.0
        self._robot.data.qacc[left_slice] = 0.0
        self._robot.data.ctrl[left_slice] = 0.0
        mujoco.mj_forward(self._robot.model, self._robot.data)

    def _validate_joint_target(self, values: Iterable[float]) -> np.ndarray:
        target = np.asarray(tuple(values), dtype=float)
        if target.shape != (self.DOF,):
            raise ValueError(f"Expected {self.DOF} right-arm joints, got {target.size}.")
        if not np.all(np.isfinite(target)):
            raise ValueError("Joint target contains NaN or infinity.")
        below = target < self._right_limits[:, 0]
        above = target > self._right_limits[:, 1]
        if np.any(below | above):
            bad = int(np.flatnonzero(below | above)[0])
            low, high = self._right_limits[bad]
            raise ValueError(
                f"Joint {bad + 1} target {target[bad]:.4f} rad is outside "
                f"[{low:.4f}, {high:.4f}] rad."
            )
        return target

    def step(self, steps: int = 1) -> None:
        if steps < 1:
            raise ValueError("steps must be at least 1")
        for _ in range(steps):
            self._freeze_left()
            self._robot.step(1)
            self._freeze_left()
            if self._robot.viewer is not None:
                self._robot.viewer.sync()
            if self._realtime:
                time.sleep(self.timestep_s)

    def state(self) -> RightArmState:
        return RightArmState(
            joint_position_rad=tuple(self._robot.get_right_joint_pos()),
            joint_velocity_rad_s=tuple(self._robot.get_right_joint_vel()),
            joint_torque_nm=tuple(self._robot.get_right_joint_tau()),
            flange_pose_xyz_quat=tuple(self._robot.get_right_cart_pose()),
            gripper_width_m=float(self._robot.get_right_gripper_pos()),
            simulation_time_s=float(self._robot.data.time),
        )

    def move_joint(
        self,
        target_rad: Iterable[float],
        *,
        duration_s: float = 1.5,
        settle_s: float = 0.2,
    ) -> RightArmState:
        target = self._validate_joint_target(target_rad)
        if duration_s <= 0 or settle_s < 0:
            raise ValueError("duration_s must be positive and settle_s cannot be negative.")
        result = self._robot.move_right_joint(
            target.tolist(), desire_time=duration_s, is_sync=False
        )
        if result != 1:
            raise RuntimeError("The vendor simulator rejected the right-arm joint move.")
        total_steps = int(np.ceil((duration_s + settle_s) / self.timestep_s))
        self.step(total_steps)
        return self.state()

    def move_pose(
        self,
        pose_xyz_quat: Iterable[float],
        *,
        duration_s: float = 1.5,
        settle_s: float = 0.2,
    ) -> RightArmState:
        pose = np.asarray(tuple(pose_xyz_quat), dtype=float)
        if pose.shape != (7,) or not np.all(np.isfinite(pose)):
            raise ValueError("Pose must contain finite [x, y, z, qx, qy, qz, qw].")
        if duration_s <= 0 or settle_s < 0:
            raise ValueError("duration_s must be positive and settle_s cannot be negative.")
        result = self._robot.move_right_pose(
            pose.tolist(), desire_time=duration_s, is_sync=False
        )
        if result != 1:
            raise ValueError("Pose is invalid or inverse kinematics did not converge.")
        total_steps = int(np.ceil((duration_s + settle_s) / self.timestep_s))
        self.step(total_steps)
        return self.state()

    def set_gripper(
        self,
        width_m: float,
        *,
        force_limit_n: float = 5.0,
        duration_s: float = 0.8,
    ) -> RightArmState:
        if not np.isfinite(width_m) or width_m < 0:
            raise ValueError("Gripper width must be a non-negative finite value in metres.")
        if force_limit_n <= 0 or duration_s <= 0:
            raise ValueError("force_limit_n and duration_s must be positive.")
        steps = int(np.ceil(duration_s / self.timestep_s))
        for _ in range(steps):
            if self._robot.set_right_gripper(width_m, tau=force_limit_n) != 1:
                raise RuntimeError("The vendor simulator rejected the gripper target.")
            self.step(1)
        return self.state()

    def forward_kinematics(
        self, joint_position_rad: Iterable[float]
    ) -> tuple[float, ...]:
        target = self._validate_joint_target(joint_position_rad)
        result, pose = self._robot.forward_kine_right(target.tolist(), tool_index=0)
        if result != 1:
            raise RuntimeError("Forward kinematics failed.")
        return tuple(float(value) for value in pose)

    def inverse_kinematics(
        self,
        pose_xyz_quat: Iterable[float],
        *,
        reference_joint_rad: Iterable[float] | None = None,
    ) -> tuple[float, ...]:
        pose = np.asarray(tuple(pose_xyz_quat), dtype=float)
        if pose.shape != (7,) or not np.all(np.isfinite(pose)):
            raise ValueError("Pose must contain finite [x, y, z, qx, qy, qz, qw].")
        reference = (
            self._validate_joint_target(reference_joint_rad)
            if reference_joint_rad is not None
            else np.asarray(self.state().joint_position_rad)
        )
        result, joints = self._robot.inverse_kine_right(
            quat_pose=pose.tolist(), ref_joint=reference.tolist(), tool_index=0
        )
        if result != 1:
            raise ValueError("Inverse kinematics did not converge for this pose.")
        return tuple(float(value) for value in joints)

    def close(self) -> None:
        robot = getattr(self, "_robot", None)
        if robot is not None:
            robot.close()

    def __enter__(self) -> "D3RightArmSim":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
