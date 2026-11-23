"""Right-arm-only safety boundary for the CVTE D3 Python SDK.

The vendor ``CArmDualBot`` exposes both arms and global controller methods.
This adapter deliberately exposes only right-arm motion.  It never calls
``set_ready``, ``set_servo_enable``, ``set_control_mode``, reset, gripper, or
any left-arm command.  A caller must establish controller state on the teach
pendant before explicitly arming motion here.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
from typing import Any, Iterable, Protocol


class D3Client(Protocol):
    """Subset of the delivered SDK used by this right-arm safety boundary."""

    def get_left_config(self) -> dict[str, Any]: ...
    def get_right_config(self) -> dict[str, Any]: ...
    def get_left_status(self) -> dict[str, Any]: ...
    def get_right_status(self) -> dict[str, Any]: ...
    def get_left_joint_pos(self) -> list[float]: ...
    def get_right_joint_pos(self) -> list[float]: ...
    def get_right_joint_vel(self) -> list[float]: ...
    def move_right_joint(
        self, targets: list[float], desire_time: float, is_sync: bool
    ) -> int: ...
    def track_right_joint(self, targets: list[float], eeff_pos: float = -1.0) -> int: ...
    def task_stop(self) -> int: ...
    def disconnect(self) -> int: ...


class SafetyViolation(RuntimeError):
    """Raised before an unsafe or unauthorised vendor command is sent."""


@dataclass(frozen=True)
class RightArmSafetyConfig:
    """Conservative limits for the initial right-arm software baseline."""

    max_joint_step_rad: float = math.radians(2.0)
    minimum_move_duration_s: float = 3.0
    require_servo_enabled: bool = True
    require_position_mode: bool = True
    tracking_enabled: bool = False
    stale_command_timeout_s: float = 0.25
    global_task_stop_verified: bool = False
    log_path: Path | None = None


@dataclass(frozen=True)
class RightArmSnapshot:
    """Minimal state used for safety gating and experiment logging."""

    timestamp_utc: str
    left_joint_position_rad: tuple[float, ...]
    right_joint_position_rad: tuple[float, ...]
    right_joint_velocity_rad_s: tuple[float, ...]
    left_status: dict[str, Any]
    right_status: dict[str, Any]
    left_joint_limits_rad: tuple[tuple[float, float], ...]
    right_joint_limits_rad: tuple[tuple[float, float], ...]


def _finite_vector(values: Iterable[float], label: str) -> tuple[float, ...]:
    vector = tuple(float(value) for value in values)
    if len(vector) != 7:
        raise SafetyViolation(f"{label} must contain exactly 7 values.")
    if not all(math.isfinite(value) for value in vector):
        raise SafetyViolation(f"{label} contains NaN or infinity.")
    return vector


def _joint_limits(config: dict[str, Any], label: str) -> tuple[tuple[float, float], ...]:
    lower = _finite_vector(config.get("limit_lower", ()), f"{label} lower limits")
    upper = _finite_vector(config.get("limit_upper", ()), f"{label} upper limits")
    limits = tuple(zip(lower, upper))
    if any(low >= high for low, high in limits):
        raise SafetyViolation(f"{label} contains an invalid joint-limit interval.")
    return limits


def _within_limits(values: Iterable[float], limits: tuple[tuple[float, float], ...]) -> bool:
    return all(low <= value <= high for value, (low, high) in zip(values, limits))


class D3RightArmSafetyAdapter:
    """Explicitly armed, right-arm-only command boundary for a D3 controller.

    The constructor has no SDK or network side effect.  Construct the vendor
    client outside the adapter or use :meth:`from_carm` only after the operator
    has selected the correct physical controller.
    """

    DOF = 7

    def __init__(self, client: D3Client, config: RightArmSafetyConfig | None = None):
        self._client = client
        self._config = config or RightArmSafetyConfig()
        self._motion_armed = False
        self._tracking_active = False
        self._last_tracking_command_monotonic: float | None = None
        self._closed = False

    @classmethod
    def from_carm(
        cls,
        address: str,
        *,
        port: int = 8090,
        timeout: float = 1.0,
        config: RightArmSafetyConfig | None = None,
    ) -> "D3RightArmSafetyAdapter":
        """Create the delivered SDK client only on this explicit call."""
        try:
            from carm import CArmDualBot
        except ImportError as exc:  # pragma: no cover - depends on delivered SDK
            raise RuntimeError("Install the delivered `carm` Python SDK first.") from exc
        return cls(
            CArmDualBot(address, port=port, timeout=timeout, left_index=0, right_index=1),
            config=config,
        )

    def _log(self, event: str, **payload: Any) -> None:
        if self._config.log_path is None:
            return
        record = {
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "event": event,
            **payload,
        }
        self._config.log_path.parent.mkdir(parents=True, exist_ok=True)
        with self._config.log_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")

    def read_snapshot(self) -> RightArmSnapshot:
        """Read feedback for diagnostics; this method never sends a command.

        Left-arm feedback is diagnostic-only.  It is included in the log so a
        later dual-arm extension has a synchronized baseline, but the adapter
        exposes no left-arm command and never evaluates a left target.
        """
        left_config = self._client.get_left_config()
        right_config = self._client.get_right_config()
        left_position = _finite_vector(self._client.get_left_joint_pos(), "left positions")
        right_position = _finite_vector(self._client.get_right_joint_pos(), "right positions")
        right_velocity = _finite_vector(self._client.get_right_joint_vel(), "right velocities")
        snapshot = RightArmSnapshot(
            timestamp_utc=datetime.now(timezone.utc).isoformat(),
            left_joint_position_rad=left_position,
            right_joint_position_rad=right_position,
            right_joint_velocity_rad_s=right_velocity,
            left_status=dict(self._client.get_left_status()),
            right_status=dict(self._client.get_right_status()),
            left_joint_limits_rad=_joint_limits(left_config, "left config"),
            right_joint_limits_rad=_joint_limits(right_config, "right config"),
        )
        self._validate_snapshot(snapshot)
        return snapshot

    def _validate_snapshot(self, snapshot: RightArmSnapshot) -> None:
        if not _within_limits(snapshot.left_joint_position_rad, snapshot.left_joint_limits_rad):
            raise SafetyViolation("left-arm feedback is outside delivered SDK limits.")
        if not _within_limits(snapshot.right_joint_position_rad, snapshot.right_joint_limits_rad):
            raise SafetyViolation("right-arm feedback is outside delivered SDK limits.")
        if self._config.require_servo_enabled and snapshot.right_status.get("servo") != 1:
            raise SafetyViolation("right-arm servo is not enabled.")
        if self._config.require_position_mode and snapshot.right_status.get("fsm_state") != "POSITION":
            raise SafetyViolation("right arm is not in POSITION mode.")
        if snapshot.right_status.get("motion_state") != 1:
            raise SafetyViolation("right arm is not motion-ready.")

    def arm_motion(self) -> RightArmSnapshot:
        """Run live preflight and explicitly permit one or more right-arm commands."""
        if self._closed:
            raise SafetyViolation("adapter is closed.")
        snapshot = self.read_snapshot()
        self._motion_armed = True
        self._log("motion_armed", right_joint_position_rad=snapshot.right_joint_position_rad)
        return snapshot

    def disarm_motion(self, reason: str) -> None:
        self._motion_armed = False
        self._tracking_active = False
        self._last_tracking_command_monotonic = None
        self._log("motion_disarmed", reason=reason)

    def _validate_target(self, target_rad: Iterable[float], snapshot: RightArmSnapshot) -> tuple[float, ...]:
        target = _finite_vector(target_rad, "right-arm target")
        if not _within_limits(target, snapshot.right_joint_limits_rad):
            raise SafetyViolation("right-arm target is outside delivered SDK limits.")
        largest_step = max(
            abs(target_value - current_value)
            for target_value, current_value in zip(target, snapshot.right_joint_position_rad)
        )
        if largest_step > self._config.max_joint_step_rad:
            raise SafetyViolation(
                "right-arm target exceeds the configured per-command joint-step limit."
            )
        return target

    def move_joint(self, target_rad: Iterable[float], *, duration_s: float) -> RightArmSnapshot:
        """Send one right-arm point-to-point command after a fresh safety check."""
        if not self._motion_armed:
            raise SafetyViolation("motion is not armed; call arm_motion() after preflight.")
        if not math.isfinite(duration_s) or duration_s < self._config.minimum_move_duration_s:
            raise SafetyViolation(
                f"duration must be at least {self._config.minimum_move_duration_s:.1f} seconds."
            )
        before = self.read_snapshot()
        target = self._validate_target(target_rad, before)
        self._log("right_move_joint_requested", target_rad=target, duration_s=duration_s)
        result = self._client.move_right_joint(list(target), duration_s, True)
        if result != 1:
            self.disarm_motion("vendor_move_right_joint_rejected")
            raise SafetyViolation(f"vendor move_right_joint returned {result!r}.")
        after = self.read_snapshot()
        self._log(
            "right_move_joint_completed",
            target_rad=target,
            duration_s=duration_s,
            right_joint_position_rad=after.right_joint_position_rad,
        )
        return after

    def track_joint(self, target_rad: Iterable[float]) -> None:
        """Transmit one right-arm tracking target when continuous control is enabled."""
        if not self._config.tracking_enabled:
            raise SafetyViolation("tracking is disabled in the safety configuration.")
        if not self._config.global_task_stop_verified:
            raise SafetyViolation("tracking requires a verified global task_stop path.")
        if not self._motion_armed:
            raise SafetyViolation("motion is not armed; call arm_motion() after preflight.")
        snapshot = self.read_snapshot()
        target = self._validate_target(target_rad, snapshot)
        result = self._client.track_right_joint(list(target), -1.0)
        if result != 1:
            self.disarm_motion("vendor_track_right_joint_rejected")
            raise SafetyViolation(f"vendor track_right_joint returned {result!r}.")
        self._tracking_active = True
        self._last_tracking_command_monotonic = time.monotonic()
        self._log("right_track_joint_sent", target_rad=target)

    def enforce_watchdog(self) -> None:
        """Stop a stale tracking stream only after global stop has been verified."""
        if not self._tracking_active or self._last_tracking_command_monotonic is None:
            return
        elapsed = time.monotonic() - self._last_tracking_command_monotonic
        if elapsed <= self._config.stale_command_timeout_s:
            return
        self._client.task_stop()
        self.disarm_motion("tracking_command_timeout")
        raise SafetyViolation("stale tracking command caused task_stop.")

    def stop_motion(self, reason: str = "operator_requested") -> None:
        """Stop the controller task and revoke this process' motion authority.

        ``task_stop`` is a controller-wide operation, so this method is only
        exposed after that path has been independently verified on the target
        robot.  It is used when teleoperation is toggled off; servo disable is
        intentionally not used because the D3 has no joint brakes.
        """
        if not self._config.global_task_stop_verified:
            raise SafetyViolation("task_stop has not been independently verified.")
        result = self._client.task_stop()
        self.disarm_motion(reason)
        self._log("task_stop_completed", reason=reason, result=result)
        if result != 1:
            raise SafetyViolation(f"vendor task_stop returned {result!r}.")

    def close(self) -> None:
        """Disconnect without enabling, resetting, or moving either arm."""
        if self._closed:
            return
        if self._tracking_active and self._config.global_task_stop_verified:
            self._client.task_stop()
            self.disarm_motion("tracking_stopped_on_close")
        self._client.disconnect()
        self._closed = True
        self._log("disconnected")

    def __enter__(self) -> "D3RightArmSafetyAdapter":
        return self

    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        self.close()
