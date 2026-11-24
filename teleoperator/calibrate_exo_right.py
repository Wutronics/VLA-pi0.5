#!/usr/bin/env python3
"""Interactively identify the Qnbot right-arm channel order and signs.

This utility opens only the exoskeleton serial device.  It never imports the
D3 SDK, connects to the robot controller, or sends a robot command.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from read_exo import ExoReader, load_config


JOINT_HINTS = (
    "J0（肩部主运动）",
    "J1（肩部主运动）",
    "J2（肩部旋转）",
    "J3（肘部屈伸）",
    "J4（前臂旋转）",
    "J5（腕部俯仰）",
    "J6（腕部侧摆/旋转）",
)


def _wait_for_data(reader: ExoReader, timeout_s: float = 5.0) -> np.ndarray:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        frame = reader.read()
        if frame is not None:
            return np.asarray(frame["action"][7:14], dtype=float)
        time.sleep(0.01)
    raise RuntimeError("5 秒内未收到外骨骼数据，请检查串口、供电和 dialout 权限")


def _sample_pose(reader: ExoReader, duration_s: float, rate_hz: float) -> np.ndarray:
    samples: list[np.ndarray] = []
    deadline = time.monotonic() + duration_s
    period_s = 1.0 / rate_hz
    while time.monotonic() < deadline:
        frame = reader.read()
        if frame is not None:
            value = np.asarray(frame["action"][7:14], dtype=float)
            if value.shape == (7,) and np.all(np.isfinite(value)):
                samples.append(value.copy())
        time.sleep(period_s)
    if not samples:
        raise RuntimeError("采样阶段没有收到有效的 7 维右臂数据")
    return np.asarray(samples)


def _capture(reader: ExoReader, prompt: str, duration_s: float, rate_hz: float) -> np.ndarray:
    input(f"\n{prompt}\n保持姿态后按 Enter 开始采样：")
    print(f"采样 {duration_s:.1f} 秒，请保持不动……")
    data = _sample_pose(reader, duration_s, rate_hz)
    median_deg = np.rad2deg(np.median(data, axis=0))
    print("右臂通道中位数（deg）：" + np.array2string(median_deg, precision=2))
    return data


def _analyse_joint(
    joint: int,
    neutral_a: np.ndarray,
    positive: np.ndarray,
    neutral_b: np.ndarray,
    negative: np.ndarray,
) -> dict:
    neutral = 0.5 * (
        np.median(neutral_a, axis=0) + np.median(neutral_b, axis=0)
    )
    pos_delta = np.median(positive, axis=0) - neutral
    neg_delta = np.median(negative, axis=0) - neutral
    span = pos_delta - neg_delta
    score = np.abs(span)
    dominant = int(np.argmax(score))
    sorted_score = np.sort(score)
    second = float(sorted_score[-2])
    dominant_score = float(score[dominant])
    separation = math.inf if second < 1e-9 else dominant_score / second
    sign = 1 if span[dominant] >= 0 else -1
    return {
        "d3_joint": joint,
        "exo_channel": dominant,
        "candidate_sign": sign,
        "positive_delta_deg": np.rad2deg(pos_delta).round(4).tolist(),
        "negative_delta_deg": np.rad2deg(neg_delta).round(4).tolist(),
        "positive_to_negative_span_deg": np.rad2deg(span).round(4).tolist(),
        "dominant_span_deg": round(float(np.rad2deg(abs(span[dominant]))), 4),
        "dominant_to_second_ratio": None if math.isinf(separation) else round(separation, 4),
    }


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="只读 Qnbot 外骨骼，标定 D3 右臂 J0~J6 的通道顺序和符号"
    )
    parser.add_argument(
        "--config",
        default="teleoperator/config/config_d3_right.yaml",
        help="D3 遥操作配置文件",
    )
    parser.add_argument("--sample-seconds", type=float, default=1.5)
    parser.add_argument("--sample-hz", type=float, default=50.0)
    parser.add_argument(
        "--output",
        default=None,
        help="结果 JSON；默认写入 artifacts/calibration/",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if args.sample_seconds <= 0 or args.sample_hz <= 0:
        raise ValueError("sample-seconds 和 sample-hz 必须为正数")

    config = load_config(args.config)
    reader = ExoReader(config["exo"])
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    output = Path(args.output or f"artifacts/calibration/d3-right-exo-{timestamp}.json")

    print("\n=== D3 右臂外骨骼通道标定（只读）===")
    print("本程序只打开外骨骼串口，不连接 D3，也不会发送运动命令。")
    print("开始前先用上位机确认并记住 D3 每个 Jn 的 '+' 物理方向，然后退出运动界面。")
    print("采样时每次只动一个人体关节，幅度约 20°~30°，其他关节尽量保持不动。")

    results: list[dict] = []
    try:
        reader.open()
        initial = _wait_for_data(reader)
        print("外骨骼连接成功，当前右臂值（deg）：" + np.array2string(np.rad2deg(initial), precision=2))
        input("\n确认 D3 不会被本脚本控制、外骨骼已正确穿戴后，按 Enter 开始：")

        for joint, hint in enumerate(JOINT_HINTS):
            print(f"\n{'=' * 62}\n标定 D3 {hint}，当前进度 {joint + 1}/7")
            neutral_a = _capture(
                reader,
                "右臂回到舒适中立位，所有关节保持不动。",
                args.sample_seconds,
                args.sample_hz,
            )
            positive = _capture(
                reader,
                f"只移动该人体关节，方向应与 D3 J{joint} 的 '+' 方向一致；幅度约 20°~30°。",
                args.sample_seconds,
                args.sample_hz,
            )
            neutral_b = _capture(
                reader,
                "回到刚才的中立位。",
                args.sample_seconds,
                args.sample_hz,
            )
            negative = _capture(
                reader,
                f"只移动该人体关节，方向应与 D3 J{joint} 的 '-' 方向一致；幅度约 20°~30°。",
                args.sample_seconds,
                args.sample_hz,
            )
            result = _analyse_joint(joint, neutral_a, positive, neutral_b, negative)
            results.append(result)
            ratio = result["dominant_to_second_ratio"]
            ratio_text = "∞" if ratio is None else f"{ratio:.2f}"
            print(
                f"候选结果：D3 J{joint} <- 外骨骼通道 {result['exo_channel']}，"
                f"sign={result['candidate_sign']:+d}，主通道/次通道={ratio_text}"
            )

    except KeyboardInterrupt:
        print("\n用户取消标定；不写入不完整结果。")
        return 130
    finally:
        reader.close()

    indices = [int(item["exo_channel"]) for item in results]
    signs = [int(item["candidate_sign"]) for item in results]
    unique = len(set(indices)) == 7
    well_separated = all(
        item["dominant_to_second_ratio"] is None
        or item["dominant_to_second_ratio"] >= 2.0
        for item in results
    )
    report = {
        "created_at": datetime.now().astimezone().isoformat(),
        "source_config": str(Path(args.config).resolve()),
        "safe_read_only": True,
        "candidate_mapping": {"exo_indices": indices, "sign": signs},
        "checks": {
            "channels_form_permutation": unique,
            "all_dominant_to_second_ratios_at_least_2": well_separated,
            "manual_review_required": True,
        },
        "joints": results,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    print(f"\n标定结果已保存：{output}")
    print(f"候选 exo_indices: {indices}")
    print(f"候选 sign:        {signs}")
    if not unique:
        print("警告：通道不是 0~6 的一一映射，至少有一个动作发生串扰或动作做错，需要重测。")
    if not well_separated:
        print("警告：部分关节主通道不够突出（比值 < 2），不要据此开启真机。")
    print("结果只用于人工复核；脚本没有修改 YAML，也没有授权真机遥操作。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
