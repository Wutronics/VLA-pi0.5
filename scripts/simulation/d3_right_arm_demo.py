#!/usr/bin/env python3
"""Small, safe-by-scope exercises for the CVTE D3 MuJoCo model."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.simulation import D3RightArmSim


HOME = np.zeros(7)
PRACTICE_TARGET = np.array([0.25, -0.45, 0.20, 0.55, 0.0, 0.20, 0.0])


def print_state(sim: D3RightArmSim, label: str) -> None:
    state = sim.state()
    joints_deg = np.rad2deg(state.joint_position_rad)
    xyz = state.flange_pose_xyz_quat[:3]
    print(f"\n[{label}]")
    print("  右臂关节角 (deg):", np.round(joints_deg, 2).tolist())
    print("  右臂法兰位置 (m):", np.round(xyz, 4).tolist())
    print(f"  右夹爪宽度 (m): {state.gripper_width_m:.4f}")
    print(f"  仿真时间 (s): {state.simulation_time_s:.3f}")


def joint_demo(sim: D3RightArmSim) -> None:
    left_before = np.asarray(sim.left_joint_position_rad)
    state = sim.move_joint(PRACTICE_TARGET, duration_s=1.5)
    error = np.max(np.abs(np.asarray(state.joint_position_rad) - PRACTICE_TARGET))
    left_drift = np.max(np.abs(np.asarray(sim.left_joint_position_rad) - left_before))
    print_state(sim, "关节运动目标")
    print(f"  最大右臂关节误差 (rad): {error:.5f}")
    print(f"  最大左臂漂移 (rad): {left_drift:.3e}")
    if error > 0.08 or left_drift > 1e-10:
        raise RuntimeError("Joint demo failed its error or left-arm isolation check.")
    sim.move_joint(HOME, duration_s=1.5)


def gripper_demo(sim: D3RightArmSim) -> None:
    opened = sim.set_gripper(0.065, duration_s=0.8)
    print(f"\n[夹爪打开] 实际宽度: {opened.gripper_width_m:.4f} m")
    closed = sim.set_gripper(0.020, duration_s=0.8)
    print(f"[夹爪闭合] 实际宽度: {closed.gripper_width_m:.4f} m")


def kinematics_demo(sim: D3RightArmSim) -> None:
    pose = np.asarray(sim.forward_kinematics(PRACTICE_TARGET))
    solution = np.asarray(
        sim.inverse_kinematics(pose, reference_joint_rad=PRACTICE_TARGET)
    )
    reconstructed = np.asarray(sim.forward_kinematics(solution))
    position_error = np.linalg.norm(reconstructed[:3] - pose[:3])
    print("\n[正/逆运动学]")
    print("  目标法兰位置 (m):", np.round(pose[:3], 4).tolist())
    print("  IK 关节解 (deg):", np.round(np.rad2deg(solution), 2).tolist())
    print(f"  FK 重建位置误差 (m): {position_error:.6f}")
    if position_error > 0.03:
        raise RuntimeError("Kinematics round-trip error is unexpectedly large.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--demo",
        choices=("state", "joint", "gripper", "kinematics", "all"),
        default="state",
        help="exercise to run (default: state)",
    )
    parser.add_argument(
        "--render", action="store_true", help="open the interactive MuJoCo viewer"
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    with D3RightArmSim(render=args.render) as sim:
        print("D3 official MuJoCo model loaded; command scope: RIGHT ARM ONLY")
        print_state(sim, "初始状态")
        if args.demo in ("joint", "all"):
            joint_demo(sim)
        if args.demo in ("gripper", "all"):
            gripper_demo(sim)
        if args.demo in ("kinematics", "all"):
            kinematics_demo(sim)
        print("\nPASS: selected simulation exercises completed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
