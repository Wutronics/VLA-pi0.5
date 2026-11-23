#!/usr/bin/env python3
"""Collect a non-motion baseline snapshot from a delivered CVTE D3 controller.

This script never calls ``set_ready``, servo/mode APIs, move APIs, track APIs,
gripper APIs, task stop, or emergency stop.  Network connection itself is
also opt-in: without ``--execute`` the script prints its plan and exits.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


MOTION_METHOD_MARKERS = (
    "move_",
    "track_",
    "set_ready",
    "set_servo",
    "set_control_mode",
    "set_right_eeff",
    "set_left_eeff",
    "set_right_gripper",
    "set_left_gripper",
    "task_stop",
    "emergency_stop",
)


def _json_value(value: Any) -> Any:
    """Convert SDK values into JSON-safe, finite primitive values."""
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("SDK returned NaN or infinity.")
        return value
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    if hasattr(value, "tolist"):
        return _json_value(value.tolist())
    return str(value)


def _require_seven(values: Any, label: str) -> list[float]:
    converted = _json_value(values)
    if not isinstance(converted, list) or len(converted) != 7:
        raise ValueError(f"{label} must contain 7 values; got {converted!r}")
    if not all(isinstance(item, (int, float)) for item in converted):
        raise ValueError(f"{label} contains a non-numeric value.")
    return [float(item) for item in converted]


def collect_snapshot(robot: Any) -> dict[str, Any]:
    """Read version, configuration, state, and feedback without commanding motion."""
    snapshot = {
        "sdk_version": _json_value(robot.get_version()),
        "left": {
            "config": _json_value(robot.get_left_config()),
            "status": _json_value(robot.get_left_status()),
            "joint_position_rad": _require_seven(
                robot.get_left_joint_pos(), "left joint positions"
            ),
            "joint_velocity_rad_s": _require_seven(
                robot.get_left_joint_vel(), "left joint velocities"
            ),
            "joint_torque_nm": _require_seven(
                robot.get_left_joint_tau(), "left joint torques"
            ),
            "flange_pose": _json_value(robot.get_left_cart_pose()),
        },
        "right": {
            "config": _json_value(robot.get_right_config()),
            "status": _json_value(robot.get_right_status()),
            "joint_position_rad": _require_seven(
                robot.get_right_joint_pos(), "right joint positions"
            ),
            "joint_velocity_rad_s": _require_seven(
                robot.get_right_joint_vel(), "right joint velocities"
            ),
            "joint_torque_nm": _require_seven(
                robot.get_right_joint_tau(), "right joint torques"
            ),
            "flange_pose": _json_value(robot.get_right_cart_pose()),
        },
    }
    for side in ("left", "right"):
        dof = snapshot[side]["config"].get("dof")
        if dof is not None and dof != 7:
            raise ValueError(f"{side} arm reports {dof} DOF, expected 7.")
    return snapshot


def motion_preflight(snapshot: dict[str, Any]) -> dict[str, Any]:
    """Identify configuration conflicts that must block a later motion command."""
    blockers: list[dict[str, Any]] = []
    for side in ("left", "right"):
        arm = snapshot[side]
        lower = arm["config"].get("limit_lower")
        upper = arm["config"].get("limit_upper")
        positions = arm["joint_position_rad"]
        if not isinstance(lower, list) or not isinstance(upper, list):
            blockers.append({"side": side, "reason": "missing_joint_limits"})
            continue
        if len(lower) != 7 or len(upper) != 7:
            blockers.append({"side": side, "reason": "invalid_joint_limit_dimension"})
            continue
        for index, (position, low, high) in enumerate(zip(positions, lower, upper)):
            if not low <= position <= high:
                blockers.append(
                    {
                        "side": side,
                        "joint_index_1based": index + 1,
                        "position_rad": position,
                        "lower_rad": low,
                        "upper_rad": high,
                        "reason": "feedback_outside_sdk_limit",
                    }
                )
    return {
        "eligible_for_motion": not blockers,
        "blockers": blockers,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="explicitly permit a read-only SDK connection; no motion is sent",
    )
    parser.add_argument(
        "--address",
        default=os.environ.get("D3_CONTROLLER_ADDRESS"),
        help="D3 controller address (or set D3_CONTROLLER_ADDRESS)",
    )
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("artifacts/evaluation/d3-hardware-readonly.json"),
        help="local result path; this directory is Git-ignored",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.execute:
        print("DRY RUN: no SDK connection will be made.")
        print("Planned read-only calls: version, left/right config, status, joint "
              "position/velocity/torque, and flange pose.")
        print("Use --execute --address <controller-address> only after the hardware "
              "safety checklist has been confirmed on site.")
        return 0
    if not args.address:
        raise SystemExit("--address or D3_CONTROLLER_ADDRESS is required with --execute.")

    try:
        from carm import CArmDualBot
    except ImportError as exc:
        raise SystemExit(
            "Cannot import the delivered `carm` SDK. Install the vendor SDK in the "
            "active Python environment before running an on-robot benchmark."
        ) from exc

    robot = CArmDualBot(
        args.address, port=args.port, timeout=args.timeout, left_index=0, right_index=1
    )
    try:
        snapshot = collect_snapshot(robot)
        preflight = motion_preflight(snapshot)
        result = {
            "benchmark": "d3-hardware-readonly",
            "executed_at_utc": datetime.now(timezone.utc).isoformat(),
            "controller_address": args.address,
            "port": args.port,
            "motion_commands_sent": False,
            "result": "pass",
            "snapshot": snapshot,
            "motion_preflight": preflight,
        }
    finally:
        robot.disconnect()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    if result["motion_preflight"]["eligible_for_motion"]:
        print(f"PASS: read-only hardware snapshot written to {args.output}")
    else:
        print(f"WARNING: read-only snapshot written to {args.output}; motion is blocked "
              "by the recorded preflight checks.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
