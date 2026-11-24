#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
外骨骼数据读取器

提供:
  action:    外骨骼→机器人 14维目标关节角
  joystick:  左右摇杆 [L_ax, L_ay, L_tr, R_ax, R_ay, R_tr]
  button:    左右按钮 [L_B0..3, R_B0..3]  (0=按下)
  red_button: 左手红色按钮是否按下 (bool)
"""

import sys
from pathlib import Path
import numpy as np
SDK_DIR = Path(__file__).resolve().parent / "qnbot_sdk_v1.2"
sys.path.insert(0, str(SDK_DIR))
from qnbot_sdk import QnbotClient, TelemetrySnapshot
try:
    from .serial_port import check_serial_access, select_serial_port
except ImportError:  # direct execution through teleoperator/main.py
    from serial_port import check_serial_access, select_serial_port
import yaml
import time
# ===== 符号转换：外骨骼 → 机器人 =====
# 外骨骼左臂正方向: 前伸(-) 外展(-) 外旋(+) 肘展开(-) 小臂外旋(-)  腕关节屈(-)  腕关节外展(-)
# 外骨骼右臂正方向: 前伸(+) 外展(+) 外旋(-) 肘展开(-) 小臂外旋(+)  腕关节屈(-)  腕关节外展(+)

# S1机器人左臂正方向: 前伸(-) 外展(+) 外旋(+) 肘展开(+) 小臂外旋(+)腕关节屈(-)  腕关节外展(+)
# S1机器人右臂正方向: 前伸(-) 外展(-) 外旋(-) 肘展开(+) 小臂外旋(-)腕关节屈(-)  腕关节外展(-)
# SIGN_FLIP_LEFT = [1, -1, 1, -1, -1, 1, -1]   # 肩俯仰, 肩横滚, 肩偏航, 肘, 小臂旋转, 腕俯仰, 腕横滚
# SIGN_FLIP_RIGHT = [-1, -1, 1, -1, -1, 1, -1]

# 轮臂机器人左臂正方向： 前伸(+) 外展(+) 外旋(+) 肘展开(+) 小臂外旋(+) 腕关节屈(-)  腕关节外展(+)
# 轮臂机器人左臂正方向： 前伸(+) 外展(+) 外旋(-) 肘展开(+) 小臂外旋(-) 腕关节屈(-)  腕关节外展(-)
# SIGN_FLIP 的逆等于自身, 因为每个元素是 ±1

# SIGN_FLIP_LEFT = [-1, -1, 1, -1, -1, 1, -1]   # 肩俯仰, 肩横滚, 肩偏航, 肘, 小臂旋转, 腕俯仰, 腕横滚
# SIGN_FLIP_RIGHT = [1, 1, 1, -1, -1, 1, -1]

# # 处理流程: (raw - 外骨骼系偏移) × SIGN_FLIP → 机器人坐标系输出
# # ZERO_OFFSETS_LEFT =  [2,   1.57, 0, -1.57, 0, 0, 0]
# # ZERO_OFFSETS_RIGHT = [-2, -1.57, 0, -1.57, 0, 0, 0]

# ZERO_OFFSETS_LEFT =  [2,  1.57, 0, -1.57, 0, 0, 0 ]
# ZERO_OFFSETS_RIGHT = [-2,  -1.57, 0, -1.57, 0, 0, 0]

# NUM_JOINTS = 7 * 2  # 14 (左右各7关节)
# RED_BUTTON = 0x0002

# ====================== 新增：关节限位【单位：弧度】======================
# 顺序：左臂7关节[0~6] → 右臂7关节[7~13]
# 格式 [[joint0_min, joint0_max], [joint1_min, joint1_max], ...]
# !!!请你根据机器人实际硬件限位修改下面数值!!!


class ExoReader:

    def __init__(self, exo_cfg: dict):
        # 从配置加载参数
        self.port = select_serial_port(str(exo_cfg.get("port", "auto")))
        self.baudrate = exo_cfg["baudrate"]
        self.RED_BUTTON = exo_cfg["red_button_mask"]

        self.SIGN_FLIP_LEFT = np.array(exo_cfg["sign_flip_left"], dtype=np.float32)
        self.SIGN_FLIP_RIGHT = np.array(exo_cfg["sign_flip_right"], dtype=np.float32)
        self.ZERO_OFFSETS_LEFT = np.array(exo_cfg["zero_offsets_left"], dtype=np.float32)
        self.ZERO_OFFSETS_RIGHT = np.array(exo_cfg["zero_offsets_right"], dtype=np.float32)
        self.CLIP_ACTIONS = bool(exo_cfg.get("clip_actions", True))


        joint_limits = exo_cfg["joint_limits"]
        self.JOINT_MIN = np.array([lim[0] for lim in joint_limits], dtype=np.float32)
        self.JOINT_MAX = np.array([lim[1] for lim in joint_limits], dtype=np.float32)
        # 加载配置之后追加
        print("==== ExoReader 加载配置参数 ====")
        print(f"port: {self.port}")
        print(f"baudrate: {self.baudrate}")
        print(f"RED_BUTTON mask: {hex(self.RED_BUTTON)}")
        print(f"SIGN_FLIP_LEFT: {self.SIGN_FLIP_LEFT}")
        print(f"SIGN_FLIP_RIGHT: {self.SIGN_FLIP_RIGHT}")
        print(f"ZERO_OFFSETS_LEFT: {self.ZERO_OFFSETS_LEFT}")
        print(f"ZERO_OFFSETS_RIGHT: {self.ZERO_OFFSETS_RIGHT}")
        print(f"JOINT_MIN:\n{self.JOINT_MIN}")
        print(f"JOINT_MAX:\n{self.JOINT_MAX}")
        print("================================\n")

        self._client = None
        self._latest = None


    def open(self):
        check_serial_access(self.port)
        self._client = QnbotClient(port=self.port, baudrate=self.baudrate)

        def _cb(snap: TelemetrySnapshot):
            self._latest = snap

        self._client.register_telemetry_callback(_cb)
        self._client.start()
        return True

    def read(self) -> dict | None:
        """读取最新一帧外骨骼数据, 返回 None 表示还没数据."""
        snap = self._latest
        if snap is None:
            return None

        # action: 外骨骼 → 机器人
        left = np.array([
            (snap.arm_joint_left_rad[i] - self.ZERO_OFFSETS_LEFT[i]) * self.SIGN_FLIP_LEFT[i]
            for i in range(7)
        ], dtype=np.float32)
        right = np.array([
            (snap.arm_joint_right_rad[i] - self.ZERO_OFFSETS_RIGHT[i]) * self.SIGN_FLIP_RIGHT[i]
            for i in range(7)
        ], dtype=np.float32)

        action = np.concatenate([left, right])
        if self.CLIP_ACTIONS:
            action = np.clip(action, self.JOINT_MIN, self.JOINT_MAX)

        # joystick
        joystick = np.array([
            snap.joystick_left[0], snap.joystick_left[1], snap.joystick_left[3],
            snap.joystick_right[0], snap.joystick_right[1], snap.joystick_right[3],
        ], dtype=np.float32)

        # button (按下=1, 松开=0)
        btn = []
        for mask in [snap.joystick_left[2], snap.joystick_right[2]]:
            for b in range(4):
                btn.append(1 - ((mask >> b) & 1))
        button = np.array(btn, dtype=np.int64)

        # red button
        red = (snap.joystick_left[2] & self.RED_BUTTON) == 0

        return {
            "action": action,
            "joystick": joystick,
            "button": button,
            "red_button": red,
        }

    def close(self):
        if self._client:
            self._client.stop()


def load_config(config_path: str) -> dict:
    cfg_file = Path(config_path)
    if not cfg_file.exists():
        raise FileNotFoundError(f"配置文件不存在: {cfg_file.resolve()}")
    with open(cfg_file, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)
    print(f"✅ 加载配置：{cfg_file.resolve()}")
    return config

def joystick_to_vel(raw_x: int, raw_y: int, dead_zone: int = 60):
    """
    外骨骼摇杆原始坐标 -> 底盘线速度v(m/s)、角速度w(rad/s)
    :param raw_x: 摇杆X轴读数
    :param raw_y: 摇杆Y轴读数
    :param dead_zone: 摇杆死区阈值，消除中位漂移抖动
    :return: (v, w)
        v: 线速度  [-1.25, 1.25]，正值前进，负值后退
        w: 角速度 [-1.05, 1.05]，正值左转，负值右转
    """
    # 摇杆标定参数
    X_MID = 2031
    Y_MID = 2032
    # X轴：左最大偏移、右最大偏移
    X_POS_MAX = 3755 - X_MID    # 1724 向左
    X_NEG_MAX = 248 - X_MID    # -1783 向右
    # Y轴：上最大偏移、下最大偏移
    Y_POS_MAX = 3848 - Y_MID   # 1816 向上(前进)
    Y_NEG_MAX = 248 - Y_MID    # -1784 向下(后退)

    V_MAX = 0.5
    W_MAX = 0.5

    dx = raw_x - X_MID
    dy = raw_y - Y_MID

    # 死区过滤
    if abs(dx) < dead_zone:
        dx = 0
    if abs(dy) < dead_zone:
        dy = 0

    # 映射线速度 v
    if dy >= 0:
        v = dy / Y_POS_MAX * V_MAX
    else:
        v = dy / abs(Y_NEG_MAX) * V_MAX

    # 映射角速度 w
    if dx >= 0:
        w = dx / X_POS_MAX * W_MAX
    else:
        w = dx / abs(X_NEG_MAX) * W_MAX

    # 限幅保护（防止极值溢出）
    v = max(min(v, V_MAX), -V_MAX)
    w = max(min(w, W_MAX), -W_MAX)

    return v, w

if __name__ == "__main__":
    config = load_config("config/config.yaml")
    recorder = ExoReader(config["exo"])
    recorder.open()

    while True:
        state = recorder.read()
        if state is not None:
            v, w = joystick_to_vel(state["joystick"][0], state["joystick"][1])
            print(f"state is {state}")
            print(f"v is {v}, w is {w}")
        time.sleep(2)
    recorder.close()
