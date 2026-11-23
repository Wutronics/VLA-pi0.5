#!/usr/bin/env python3
"""Execute one bounded, right-arm-only D3 joint-step benchmark.

This is deliberately not a general robot controller.  It commands only one
right-arm joint, derives the target from the live feedback, and refuses to
move when the read-only motion preflight, servo state, or SDK limits fail.
It never calls ready, servo-enable, control-mode, gripper, left-arm, or reset
APIs.  The operator must establish the safe controller state on the pendant.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

try:  # Supports both `python path/to/script.py` and package imports in tests.
    from .d3_hardware_readonly_benchmark import collect_snapshot, motion_preflight
except ImportError:  # pragma: no cover - exercised by the direct CLI entrypoint
    from d3_hardware_readonly_benchmark import collect_snapshot, motion_preflight


MAX_DELTA_DEG = 2.0
MIN_DURATION_S = 3.0


def plan_joint_step(
    snapshot: dict[str, Any], *, joint_index_1based: int, delta_deg: float
) -> list[float]:
    """Validate the live state and return a target that differs at one joint."""
    if not 1 <= joint_index_1based <= 7:
        raise ValueError("joint index must be in [1, 7].")
    if not math.isfinite(delta_deg) or not 0 < abs(delta_deg) <= MAX_DELTA_DEG:
        raise ValueError(f"delta must be finite, non-zero, and at most {MAX_DELTA_DEG} deg.")

    preflight = motion_preflight(snapshot)
    if not preflight["eligible_for_motion"]:
        raise ValueError(f"motion preflight blocked: {preflight['blockers']}")

    status = snapshot["right"]["status"]
    if status.get("servo") != 1:
        raise ValueError("right-arm servo is not enabled.")
    if status.get("fsm_state") != "POSITION":
        raise ValueError("right arm is not in POSITION mode.")
    if status.get("motion_state") != 1:
        raise ValueError("right arm is not in an active motion-ready state.")

    index = joint_index_1based - 1
    start = list(snapshot["right"]["joint_position_rad"])
    target = start.copy()
    target[index] += math.radians(delta_deg)
    lower = snapshot["right"]["config"]["limit_lower"]
    upper = snapshot["right"]["config"]["limit_upper"]
    if not lower[index] <= target[index] <= upper[index]:
        raise ValueError(
            f"J{joint_index_1based} target {target[index]:.6f} rad is outside "
            f"[{lower[index]:.6f}, {upper[index]:.6f}] rad."
        )
    return target


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="permit one motion command")
    parser.add_argument(
        "--confirm-right-only-joint-step",
        action="store_true",
        help="acknowledge that the right arm is clear, observed, and at minimum speed",
    )
    parser.add_argument(
        "--address", default=os.environ.get("D3_CONTROLLER_ADDRESS"),
        help="D3 controller address (or set D3_CONTROLLER_ADDRESS)",
    )
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--joint-index", type=int, default=1, help="1-based right-arm joint")
    parser.add_argument("--delta-deg", type=float, default=2.0)
    parser.add_argument("--duration-s", type=float, default=3.0)
    parser.add_argument("--tolerance-deg", type=float, default=0.5)
    parser.add_argument(
        "--output", type=Path,
        default=Path("artifacts/evaluation/d3-right-joint-step.json"),
        help="local result path; Git ignores this directory",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.duration_s < MIN_DURATION_S:
        raise SystemExit(f"duration must be at least {MIN_DURATION_S} seconds.")
    if args.tolerance_deg <= 0:
        raise SystemExit("tolerance must be positive.")
    if not args.execute:
        print("DRY RUN: no connection or motion will be attempted.")
        print(f"Planned command: right J{args.joint_index} by {args.delta_deg:+.2f} deg "
              f"over {args.duration_s:.1f} s.")
        return 0
    if not args.confirm_right_only_joint_step:
        raise SystemExit("Refusing motion: add --confirm-right-only-joint-step.")
    if not args.address:
        raise SystemExit("--address or D3_CONTROLLER_ADDRESS is required with --execute.")

    try:
        from carm import CArmDualBot
    except ImportError as exc:
        raise SystemExit("The delivered `carm` SDK is required for this benchmark.") from exc

    robot = CArmDualBot(
        args.address, port=args.port, timeout=args.timeout, left_index=0, right_index=1
    )
    try:
        before = collect_snapshot(robot)
        target = plan_joint_step(
            before, joint_index_1based=args.joint_index, delta_deg=args.delta_deg
        )
        result_code = robot.move_right_joint(target, args.duration_s, True)
        after = collect_snapshot(robot)
    finally:
        robot.disconnect()

    index = args.joint_index - 1
    target_error_rad = after["right"]["joint_position_rad"][index] - target[index]
    left_drift_rad = max(
        abs(after_value - before_value)
        for after_value, before_value in zip(
            after["left"]["joint_position_rad"], before["left"]["joint_position_rad"]
        )
    )
    passed = (
        result_code == 1
        and abs(math.degrees(target_error_rad)) <= args.tolerance_deg
        and motion_preflight(after)["eligible_for_motion"]
    )
    result = {
        "benchmark": "d3-right-joint-step",
        "executed_at_utc": datetime.now(timezone.utc).isoformat(),
        "controller_address": args.address,
        "motion_commands_sent": True,
        "side": "right",
        "joint_index_1based": args.joint_index,
        "delta_deg": args.delta_deg,
        "duration_s": args.duration_s,
        "sdk_return_code": result_code,
        "target_rad": target,
        "target_error_deg": math.degrees(target_error_rad),
        "max_left_drift_rad": left_drift_rad,
        "before": before,
        "after": after,
        "result": "pass" if passed else "fail",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    if not passed:
        raise SystemExit(f"FAIL: benchmark result written to {args.output}")
    print(f"PASS: right J{args.joint_index} step benchmark written to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
