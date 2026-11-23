#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
人形机器人关节控制 (Fourier GR 系列 / Aurora SDK)

两种控制模式:
  1) 阻塞模式: send_joints()   — 单次发送 + 插值平滑, 调用阻塞 ~50ms
  2) 流式模式: start_streaming() + set_target() — 后台线程持续发送, 主线程不阻塞

流式模式用法 (推荐用于遥操作):
  ctrl = RobotController(domain_id=188, robot_name="gr3")
  ctrl.open()
  ctrl.start_streaming(frequency=100)
  ctrl.set_target(left=[0.0]*7, right=[0.0]*7)  # 不阻塞, 各7关节
  ...
  ctrl.stop_streaming()
  ctrl.close()
"""

import time
import threading
import numpy as np

try:
    from fourier_aurora_client import AuroraClient
    HAS_SDK = True
except ImportError:
    HAS_SDK = False


def _interpolate(init_pos, target_pos, step, total_steps):
    return [i + (t - i) * step / total_steps for i, t in zip(init_pos, target_pos)]


class RobotController:
    def __init__(self, domain_id: int = 175, robot_name: str = "gr3"):
        self._domain_id = domain_id
        self._robot_name = robot_name
        self._client = None

        # 流式控制
        self._streaming = False
        self._stream_thread = None
        self._stream_freq = 100
        self._max_velocity = 2   # 关节最大角速度 (rad/s)
        self._target_lock = threading.Lock()
        self._latest_left = None    # 外骨骼目标
        self._latest_right = None
        self._sent_left = None      # 上一次实际发送值
        self._sent_right = None

    # ═══════════════════════════════════════════════════════
    # 连接 / 断开
    # ═══════════════════════════════════════════════════════
    def open(self, switch_to_stand: bool = True) -> bool:
        """连接机器人, 可选切换到 User Command 状态 (FSM=10)."""
        if not HAS_SDK:
            print("[RobotController] SDK 未安装, 使用占位模式")
            return True

        self._client = AuroraClient.get_instance(
            domain_id=self._domain_id,
            robot_name=self._robot_name,
        )
        if switch_to_stand:
            time.sleep(1)
            self._client.set_fsm_state(2)  # User Command State
        print(f"[RobotController] 已连接 {self._robot_name},mode3")
        return True

    def close(self):
        """断开连接 (停止流式控制, 切回 PdStand, 释放资源)."""
        self.stop_streaming()
        if self._client:
            try:
                self._client.set_fsm_state(2)  # 切回 PdStand
                time.sleep(0.3)
            except Exception:
                pass
            self._client.close()

    # ═══════════════════════════════════════════════════════
    # 阻塞模式: 单次发送 + 插值
    # ═══════════════════════════════════════════════════════
    def send_joints(self, left: list, right: list, duration: float = 0.05,
                    frequency: float = 100):
        """发送左右臂目标关节角, 带线性插值 (阻塞, 耗时 ≈ duration)."""
        if not HAS_SDK or not self._client:
            return

        # #上肢切换到PD模式进行关节角度控制
        # self._client.set_upper_fsm_state(2)
        # time.sleep(1)

        total_steps = max(1, int(frequency * duration))

        init_left = self._client.get_group_state("left_manipulator", key="position")
        init_right = self._client.get_group_state("right_manipulator", key="position")

        diff_left = [l - i for l, i in zip(left, init_left)]
        diff_right = [r - i for r, i in zip(right, init_right)]
        print(f"初始关节角度差值: left={diff_left}, right={diff_right}")

        for step in range(total_steps + 1):
            pos = {
                "left_manipulator": _interpolate(init_left, left, step, total_steps),
                "right_manipulator": _interpolate(init_right, right, step, total_steps),
            }
            self._client.set_group_cmd(pos)
            time.sleep(1 / frequency)

    # ═══════════════════════════════════════════════════════
    # 流式模式: 后台线程持续发送, 主线程不阻塞
    # ═══════════════════════════════════════════════════════
    def start_streaming(self, frequency: float = 100):
        """启动后台控制线程, 以固定频率发送最新的 set_target 值."""
        if self._streaming:
            return
        self._stream_freq = frequency
        self._streaming = True
        self._stream_thread = threading.Thread(
            target=self._streaming_loop,
            daemon=True,
        )
        self._stream_thread.start()
        print(f"[RobotController] 流式控制已启动 ({frequency}Hz)")

    def stop_streaming(self):
        """停止后台控制线程."""
        self._streaming = False
        if self._stream_thread is not None:
            self._stream_thread.join(timeout=2.0)
            self._stream_thread = None
            print("[RobotController] 流式控制已停止")

    def set_target(self, left: list, right: list):
        """更新目标关节角 (非阻塞, 仅更新内部变量)."""
        with self._target_lock:
            self._latest_left = list(left)
            self._latest_right = list(right)

    def _streaming_loop(self):
        """后台线程: 持续以固定频率发送最新目标到机器人, 带关节速度限制."""
        while self._streaming:
            with self._target_lock:
                left = list(self._latest_left) if self._latest_left is not None else None
                right = list(self._latest_right) if self._latest_right is not None else None

            if left is not None and right is not None and self._client is not None:
                try:
                    # 首次发送: 从机器人当前位置起始
                    if self._sent_left is None:
                        self._sent_left = list(
                            self._client.get_group_state("left_manipulator", "position")
                        )
                        self._sent_right = list(
                            self._client.get_group_state("right_manipulator", "position")
                        )

                    # 每步最大关节变化量
                    max_delta = self._max_velocity / self._stream_freq

                    # 左臂: 逐关节限速
                    for i in range(len(left)):
                        lo = self._sent_left[i] - max_delta
                        hi = self._sent_left[i] + max_delta
                        self._sent_left[i] = max(lo, min(hi, left[i]))

                    # 右臂: 逐关节限速
                    for i in range(len(right)):
                        lo = self._sent_right[i] - max_delta
                        hi = self._sent_right[i] + max_delta
                        self._sent_right[i] = max(lo, min(hi, right[i]))

                    self._client.set_group_cmd({
                        "left_manipulator": self._sent_left,
                        "right_manipulator": self._sent_right,
                    })
                except Exception:
                    pass  # 忽略偶发通信错误, 下一帧重试

            time.sleep(1 / self._stream_freq)

    def read(self, has_hand = False) -> np.ndarray:
        """读取机器人全部关节角，返回 ndarray[26]
        (左臂7 + 右臂7 + 左手6 + 右手6).
        如果机器人没有手部关节组，手部部分返回全零。
        """
        if HAS_SDK and self._client:
            try:
                left = self._client.get_group_state("left_manipulator", "position")
                right = self._client.get_group_state("right_manipulator", "position")
                if has_hand:
                    left_hand = self._client.get_group_state("left_hand", "position")
                    right_hand = self._client.get_group_state("right_hand", "position")
            except Exception:
                if has_hand:
                    left_hand, right_hand = [], []
            if has_hand:
                joints = np.array(
                    list(left) + list(right) + list(left_hand) + list(right_hand),
                    dtype=np.float32,
                )
            else:
                joints = np.array(
                    list(left) + list(right),
                    dtype=np.float32,
                )
        else:
            if has_hand:
                joints = np.zeros(14, dtype=np.float32)
            else:
                joints = np.zeros(26, dtype=np.float32)
        return joints
