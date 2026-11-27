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
import time
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


def _positive_vector7(value: float | Iterable[float], label: str) -> np.ndarray:
    """Accept one positive scalar or seven positive per-joint values."""
    if np.isscalar(value):
        vector = np.full(7, float(value), dtype=np.float64)
    else:
        vector = _vector7(value, label)
    if not np.isfinite(vector).all() or (vector <= 0).any():
        raise ValueError(f"{label} values must be finite and positive")
    return vector


def _shortest_angular_delta(current: np.ndarray, reference: np.ndarray) -> np.ndarray:
    """Return encoder deltas in [-pi, pi), avoiding wrap-around jumps."""
    return (current - reference + math.pi) % (2.0 * math.pi) - math.pi


class D3RightArmController(BaseRobotController):
    """Right-arm-only controller with dry-run as the default mode."""

    requires_home = False
    alignment_moves_robot = False
    EXECUTE_CONFIRMATION = "I_UNDERSTAND_D3_RIGHT_ARM_ONLY"

    def __init__(self, robot_cfg: dict):
        self.cfg = dict(robot_cfg)
        self.dry_run = bool(self.cfg.get("dry_run", True))
        legacy_step_deg = float(self.cfg.get("max_joint_step_deg", 1.0))
        self._max_command_step_rad = math.radians(
            float(self.cfg.get("max_command_step_deg", legacy_step_deg))
        )
        self._max_feedback_error_rad = math.radians(
            float(self.cfg.get("max_feedback_error_deg", legacy_step_deg))
        )
        if (
            not math.isfinite(self._max_command_step_rad)
            or self._max_command_step_rad <= 0
        ):
            raise ValueError("max_command_step_deg must be finite and positive")
        if (
            not math.isfinite(self._max_feedback_error_rad)
            or self._max_feedback_error_rad <= 0
        ):
            raise ValueError("max_feedback_error_deg must be finite and positive")
        self._max_velocity_rad_s = np.deg2rad(
            _positive_vector7(self.cfg.get("max_velocity_deg_s", 5.0), "max_velocity_deg_s")
        )
        self._max_acceleration_rad_s2 = np.deg2rad(
            _positive_vector7(
                self.cfg.get("max_acceleration_deg_s2", 20.0),
                "max_acceleration_deg_s2",
            )
        )
        configured_excursion = self.cfg.get("max_reference_delta_deg")
        self._max_reference_delta_rad: np.ndarray | None = None
        if configured_excursion is not None:
            self._max_reference_delta_rad = np.deg2rad(
                _positive_vector7(configured_excursion, "max_reference_delta_deg")
            )
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
        self._latest_velocity = np.zeros(7, dtype=np.float64)
        self._last_command_time: float | None = None
        self._last_rate_limit_report = 0.0

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
            max_joint_step_rad=self._max_feedback_error_rad,
            minimum_move_duration_s=float(self.cfg.get("minimum_move_duration_s", 3.0)),
            tracking_enabled=True,
            stale_command_timeout_s=float(self.cfg.get("stale_command_timeout_ms", 250)) / 1000.0,
            tracking_status_interval_s=float(
                self.cfg.get("tracking_status_interval_ms", 200)
            ) / 1000.0,
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
        self._latest_velocity.fill(0.0)
        self._last_command_time = None
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
            # A previous red-button stop disarms the adapter.  Re-entering
            # teleoperation must therefore repeat the live state preflight.
            snapshot = self._adapter.arm_motion()
            self._robot_reference = np.asarray(snapshot.right_joint_position_rad, dtype=np.float64)
        self._latest_command = self._robot_reference.copy()
        self._latest_velocity.fill(0.0)
        self._last_command_time = None
        print("[D3] 已捕获相对映射参考姿态；机器人未运动")

    def _map_target(self, right_target: Iterable[float]) -> np.ndarray:
        if self._exo_reference is None or self._robot_reference is None:
            raise SafetyViolation("capture a D3/exoskeleton reference before teleoperation")
        exo = self._selected_exo(right_target)
        delta = _shortest_angular_delta(exo, self._exo_reference)
        mapped_delta = delta * self._sign * self._gain
        if self._max_reference_delta_rad is not None:
            mapped_delta = np.clip(
                mapped_delta,
                -self._max_reference_delta_rad,
                self._max_reference_delta_rad,
            )
        return self._robot_reference + mapped_delta

    def _limit_target(self, requested: np.ndarray, now: float) -> np.ndarray:
        """Apply acceleration, velocity, and per-command displacement limits."""
        if self._last_command_time is None:
            # Use the configured stream period for the first command instead
            # of an arbitrarily small wall-clock interval.
            dt_s = 1.0 / float(self.cfg.get("stream_frequency_hz", 10.0))
        else:
            dt_s = min(max(now - self._last_command_time, 0.001), 0.25)

        error = requested - self._latest_command
        desired_velocity = np.clip(
            error / dt_s,
            -self._max_velocity_rad_s,
            self._max_velocity_rad_s,
        )
        max_velocity_change = self._max_acceleration_rad_s2 * dt_s
        velocity = self._latest_velocity + np.clip(
            desired_velocity - self._latest_velocity,
            -max_velocity_change,
            max_velocity_change,
        )
        step = np.clip(
            velocity * dt_s,
            -self._max_command_step_rad,
            self._max_command_step_rad,
        )

        # Snap only when the proposed step approaches the target from the
        # correct direction.  This avoids an acceleration-bypassing snap when
        # a target reverses while the arm is still decelerating.
        same_direction = step * error >= 0.0
        reached = same_direction & (np.abs(step) >= np.abs(error))
        step[reached] = error[reached]
        velocity[reached] = 0.0

        self._latest_velocity = velocity
        self._last_command_time = now
        return self._latest_command + step

    def set_joint_target(self, left_target, right_target, duration, frequency):
        del left_target, duration, frequency
        requested = self._map_target(right_target)
        if not np.isfinite(requested).all():
            raise SafetyViolation("mapped D3 target contains NaN or infinity")
        if self._joint_limits is not None:
            if (
                (requested < self._joint_limits[:, 0])
                | (requested > self._joint_limits[:, 1])
            ).any():
                raise SafetyViolation("mapped D3 target is outside configured joint limits")

        delta = requested - self._latest_command
        target = self._limit_target(requested, time.monotonic())
        if not np.allclose(target, requested, atol=1e-12):
            now = time.monotonic()
            if now - self._last_rate_limit_report >= 1.0:
                largest_requested_deg = math.degrees(float(np.max(np.abs(delta))))
                print(
                    "[D3] 目标变化已限速："
                    f"请求最大 {largest_requested_deg:.2f}°，"
                    f"速度上限 {math.degrees(float(np.max(self._max_velocity_rad_s))):.1f}°/s"
                )
                self._last_rate_limit_report = now
        if self._adapter is not None:
            target = np.asarray(
                self._adapter.track_joint(target.tolist()), dtype=np.float64
            )
        self._latest_command = target.copy()

    def get_joint_state(self) -> np.ndarray:
        if self._adapter is None:
            return self._dry_state.astype(np.float32, copy=True)
        snapshot = self._adapter.read_snapshot()
        return np.asarray(snapshot.right_joint_position_rad, dtype=np.float32)

    def action_for_dataset(self, exo_action: np.ndarray) -> np.ndarray:
        del exo_action
        return self._latest_command.astype(np.float32, copy=True)

    def telemetry_snapshot(self) -> dict[str, object]:
        """Return a read-only health snapshot for the local dashboard.

        This is called by the collector at the configured dashboard rate, never
        by the browser.  It intentionally does not call any motion API.
        """
        command = self._latest_command.astype(float).tolist()
        result: dict[str, object] = {
            "connection": "dry_run" if self._adapter is None else "connected",
            "dry_run": self.dry_run,
            "command_joint_rad": command,
            "feedback_joint_rad": self._dry_state.astype(float).tolist(),
            "limit_margin_rad": [],
            "tracking_reference_captured": (
                self._exo_reference is not None and self._robot_reference is not None
            ),
        }
        if self._adapter is None:
            return result
        try:
            snapshot = self._adapter.read_snapshot()
            feedback = np.asarray(snapshot.right_joint_position_rad, dtype=np.float64)
            result["feedback_joint_rad"] = feedback.tolist()
            result["right_status"] = snapshot.right_status
            result["left_status"] = snapshot.left_status
            if self._joint_limits is not None:
                margin = np.minimum(
                    feedback - self._joint_limits[:, 0],
                    self._joint_limits[:, 1] - feedback,
                )
                result["limit_margin_rad"] = margin.astype(float).tolist()
        except Exception as exc:
            # Surface diagnostic state without allowing dashboard polling to
            # interrupt teleoperation.  The safety adapter remains authority.
            result["connection"] = "error"
            result["error"] = str(exc)
        return result

    def stop_motion(self, reason: str = "operator_requested"):
        if self._adapter is not None:
            self._adapter.stop_motion(reason)
        self._exo_reference = None
        self._robot_reference = None
        self._latest_velocity.fill(0.0)
        self._last_command_time = None

    def tick(self):
        if self._adapter is not None:
            self._adapter.enforce_watchdog()

    def set_speed(self, v, w):
        del v, w  # D3 fixed-base platform has no mobile-base command.
