#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
外骨骼遥操作+数据集录制
主线程：状态机、按钮检测、机器人运动下发
录制子线程：独立完成传感器采集、图像读取、add_frame，自主控制录制帧率
"""

import argparse
import time
import threading
from pathlib import Path
from datetime import datetime
import cv2
import yaml
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from read_exo import ExoReader
# from read_robot import RobotReader
from robot_base import BaseRobotController
import numpy as np


def create_robot_controller(robot_cfg: dict) -> BaseRobotController:
    robot_type = robot_cfg["type"]
    if robot_type == "xoip":
        from robot_impl.xoip_robot import XoipRobotController
        return XoipRobotController(robot_cfg)
    # 后续扩展示例
    elif robot_type == "gr3":
        from robot_impl.gr3_robot import Gr3RobotController
        return Gr3RobotController(robot_cfg)
    elif robot_type == "d3_right":
        from robot_impl.d3_right_robot import D3RightArmController
        return D3RightArmController(robot_cfg)
    # elif robot_type == "can":
    #     return CanRobotController(robot_cfg)
    else:
        raise ValueError(f"不支持的机器人类型: {robot_type}")


class ExoTeleopRecorder:
    # 内部状态枚举
    class State:
        IDLE = "IDLE"           # 未归位
        HOME = "HOME"           # 归位完成
        TELEOP = "TELEOP"       # 遥操作开启
        RECORDING = "RECORDING" # 遥操作+录制

    def __init__(self, config_path: str = "config/config.yaml"):
        # 加载配置
        self.config = self._load_config(config_path)
        # 缓存名称列表
        self.name_library = self.config["name_lists"]
        self.feature_specs = self.config["feature_specs"]
        # ========== 配置参数 ==========
        self.DATASET_NAME = self.config["DATASET_NAME"]
        self.OUTPUT_ROOT = self.config["OUTPUT_ROOT"]
        self.FPS = self.config["FPS"]
        self.NUM_JOINTS = self.config["NUM_JOINTS"]
        self.CAMERA_WIDTH = self.config["CAMERA_WIDTH"]
        self.CAMERA_HEIGHT = self.config["CAMERA_HEIGHT"]
        self.CAMERA_INDICES = self.config["CAMERA_INDICES"]
        self.STREAM_FREQ = float(self.config.get("STREAM_FREQ", 10))
        if self.STREAM_FREQ <= 0:
            raise ValueError("STREAM_FREQ must be positive")
        # self.SERIAL_PORT = self.config["SERIAL_PORT"]

        self.BTN_L_WHITE = self.config["BTN_L_WHITE"]
        self.BTN_L_BLUE = self.config["BTN_L_BLUE"]

        self.HOME_LEFT = self.config.get("HOME_LEFT", [0.0] * 7)
        self.HOME_RIGHT = self.config.get("HOME_RIGHT", [0.0] * 7)
        self.ROBOT_TYPE = self.config.get("ROBOT_TYPE", self.config["robot"]["type"])

        # ========== 业务状态变量 ==========
        self.state = self.State.IDLE
        self.prev_teleop_btn = False
        self.prev_record_btn = False
        self.prev_white_btn = False

        self.episode_idx = 0
        self.frame_count = 0
        self.t0 = 0.0
        self.add_frame_times = []
        self.last_report = 0.0

        # 硬件句柄
        self.exo = None
        self.robot_ctrl = None
        # self.robot_reader = None
        self.caps = dict()
        self.has_video = False
        self.dataset = None

        # ========== 录制线程相关 ==========
        self._recorder_thread = None
        self._thread_running = False        # 线程总运行开关
        self._is_recording = False          # 录制使能（主线程控制）
        self._record_last_tick = 0.0

        # 外骨骼数据缓存 + 线程锁（解决串口多线程争抢问题）
        self._exo_lock = threading.Lock()
        self._latest_exo_data = None

    def _load_config(self, config_path: str) -> dict:
        cfg_file = Path(config_path)
        if not cfg_file.exists():
            raise FileNotFoundError(f"配置文件不存在: {cfg_file.resolve()}")
        with open(cfg_file, "r", encoding="utf-8") as f:
            config = yaml.safe_load(f)
        print(f"✅ 加载配置：{cfg_file.resolve()}")
        return config

    def _init_cameras(self) -> dict:
        caps = {}
        for idx in self.CAMERA_INDICES:
            cap = cv2.VideoCapture(idx)
            if not cap.isOpened():
                print(f"[摄像头] /dev/video{idx} 打开失败，跳过")
                continue
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.CAMERA_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.CAMERA_HEIGHT)
            cap.set(cv2.CAP_PROP_FPS, 30)
            # 预热
            for _ in range(5):
                cap.read()
            ret, frame = cap.read()
            if not ret:
                cap.release()
                continue
            caps[f"observation.images.camera_{idx}"] = (cap, (frame.shape[1], frame.shape[0]))
            print(f"[摄像头] video{idx} 就绪 {frame.shape[1]}×{frame.shape[0]}")
        return caps

    def _build_features(self) -> dict:
        features = {}
        # 遍历配置里所有特征定义
        for feat_key, spec in self.feature_specs.items():
            ref_key = spec["ref_name_list"]
            name_array = self.name_library[ref_key]
            features[feat_key] = {
                "dtype": spec["dtype"],
                "shape": (len(name_array),),
                "names": name_array
            }
        # 自动追加相机video特征
        for cam_name, (_, (w, h)) in self.caps.items():
            features[cam_name] = {
                "dtype": "video",
                "shape": (h, w, 3),
                "names": ["height", "width", "channel"]
            }
        return features

    def init_hardware(self):
        print("\n[硬件初始化]")
        self.exo = ExoReader(exo_cfg = self.config["exo"])
        self.exo.open()
        time.sleep(0.5)
        # self.robot_ctrl = RobotController()
        self.robot_ctrl = create_robot_controller(self.config["robot"])
        self.robot_ctrl.open(10)
        if not self.robot_ctrl.requires_home:
            self.state = self.State.HOME
        # self.robot_reader = RobotReader()
        self.caps = self._init_cameras()
        self.has_video = len(self.caps) > 0
        print("[硬件初始化] 完成\n")

    def release_hardware(self):
        print("\n[释放硬件资源]")
        try:
            if self.exo:
                self.exo.close()
            if self.robot_ctrl:
                self.robot_ctrl.close()
            # if self.robot_reader:
            #     self.robot_reader.close()
            for cap, _ in self.caps.values():
                cap.release()
            print("硬件全部关闭")
        except Exception as e:
            print(f"释放异常: {e}")

    def validate_frame_data(self, frame_data: dict):
        for feat_key, spec in self.feature_specs.items():
            ref_key = spec["ref_name_list"]
            expect_len = len(self.name_library[ref_key])
            arr = frame_data[feat_key]
            if len(arr) != expect_len:
                raise ValueError(
                    f"【维度不匹配】{feat_key} 期望长度:{expect_len}, 实际:{len(arr)}"
                )

    # ===================== 录制线程主循环 =====================
    def _recorder_loop(self):
        print("[录制子线程] 启动成功")
        sample_dt = 1.0 / self.FPS
        self._record_last_tick = time.perf_counter()
        while self._thread_running:
            now = time.perf_counter()
            if self._is_recording:
                next_tick = self._record_last_tick + sample_dt
                if now >= next_tick:
                    try:
                        robot_joints = self.robot_ctrl.get_joint_state()
                        if len(robot_joints) != self.NUM_JOINTS:
                            raise ValueError(
                                f"【get_joint_state 维度不匹配】期望长度:{self.NUM_JOINTS}, 实际:{len(robot_joints)}"
                            )
                        with self._exo_lock:
                            exo_data = self._latest_exo_data
                        if exo_data is not None:
                            frame_data = {
                                "observation.state": robot_joints,
                                "action": self.robot_ctrl.action_for_dataset(exo_data["action"]),
                                "observation.joystick": exo_data["joystick"],
                                "observation.button": exo_data["button"],
                                "task": "teleop",
                            }
                            for cam_name, (cap, _) in self.caps.items():
                                ret, frame = cap.read()
                                if ret:
                                    frame_data[cam_name] = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                            try:
                                self.validate_frame_data(frame_data)
                            except ValueError as err:
                                print(f"[帧数据校验失败] {err}")
                                continue
                            t_add = time.perf_counter()
                            self.dataset.add_frame(frame_data)
                            cost_ms = (time.perf_counter() - t_add) * 1000
                            self.add_frame_times.append(cost_ms)
                            self.frame_count += 1
                        # 关键：使用理论时刻推进，不用当前now
                        self._record_last_tick = next_tick
                    except Exception as e:
                        print(f"[录制线程异常] {e}")
            # 短休眠降低CPU占用
            time.sleep(0.0005)

    def start_recorder_thread(self):
        """启动录制线程"""
        self._thread_running = True
        self._recorder_thread = threading.Thread(target=self._recorder_loop, daemon=True)
        self._recorder_thread.start()

    def stop_recorder_thread(self):
        """停止录制线程，等待回收"""
        self._thread_running = False
        if self._recorder_thread and self._recorder_thread.is_alive():
            self._recorder_thread.join()

    def _print_banner(self):
        print(f"\n{'='*60}")
        print(f"  外骨骼遥操作 + 子线程异步录制")
        print(f"{'='*60}")
        print(f"⚪白色按键：归位 | 🔴红色按键：遥操作开关 | 🔵蓝色按键：录制开关")
        print(f"Ctrl+C 安全退出程序\n")

    def joystick_to_vel(self, raw_x: int, raw_y: int, dead_zone: int = 60):
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

    def run(self):
        self.init_hardware()
        self.start_recorder_thread()
        self._print_banner()
        features = self._build_features()

        try:
            while True:
                exo_data = self.exo.read()

                self.robot_ctrl.tick()
                if exo_data is None:
                    time.sleep(0.001)
                    continue
                now = time.perf_counter()

                # 更新全局外骨骼缓存（线程安全）
                with self._exo_lock:
                    self._latest_exo_data = exo_data

                teleop_btn = exo_data["red_button"]
                record_btn = bool(exo_data["button"][self.BTN_L_BLUE])
                white_btn = bool(exo_data["button"][self.BTN_L_WHITE])


                # ========== 白色按钮：归位 ==========
                if white_btn and not self.prev_white_btn:
                    if not self.robot_ctrl.requires_home:
                        print("[⚪] D3 右臂模式不执行自动归位")
                    elif self.state in (self.State.IDLE, self.State.HOME):
                        was_idle = (self.state == self.State.IDLE)
                        print(f"\n[⚪] 执行归位")
                        try:
                            self.robot_ctrl.set_joint_target(self.HOME_LEFT, self.HOME_RIGHT, duration=2.0, frequency=100)
                            if was_idle:
                                self.state = self.State.HOME
                                print("[⚪] 归位完成 → HOME状态")
                        except Exception as e:
                            print(f"归位失败：{e}")
                    else:
                        print("[⚪] ⚠ 请先退出遥操作再归位")
                self.prev_white_btn = white_btn

                # ========== 红色按钮：遥操作开关 ==========
                if teleop_btn and not self.prev_teleop_btn:
                    if self.state == self.State.IDLE:
                        print("[🔴] ⚠ 先归位！")
                    elif self.state == self.State.HOME:
                        action = exo_data["action"]
                        left_target = action[:7].tolist()
                        right_target = action[7:].tolist()
                        if self.robot_ctrl.alignment_moves_robot:
                            print("[🔴] 平滑对齐初始姿态...")
                        else:
                            print("[🔴] 捕获外骨骼/机器人参考姿态...")
                        self.robot_ctrl.align_for_teleop(
                            left_target, right_target, duration=2.0, frequency=100
                        )
                        if self.robot_ctrl.alignment_moves_robot:
                            time.sleep(3)
                        self.state = self.State.TELEOP
                        print("[🔴] 进入遥操作")
                    elif self.state == self.State.TELEOP:
                        self.robot_ctrl.stop_motion("red_button_teleop_exit")
                        self.state = (
                            self.State.HOME
                            if not self.robot_ctrl.requires_home
                            else self.State.IDLE
                        )
                        print("[🔴] 退出遥操作")
                    elif self.state == self.State.RECORDING:
                        # 停止录制
                        self._is_recording = False
                        if self.frame_count > 0:
                            self.dataset.save_episode()
                            print(f"\n[■] Episode {self.episode_idx} 保存，总帧数：{self.frame_count}")
                            self.episode_idx += 1
                        self.frame_count = 0
                        self.robot_ctrl.stop_motion("red_button_recording_exit")
                        self.state = (
                            self.State.HOME
                            if not self.robot_ctrl.requires_home
                            else self.State.IDLE
                        )
                        print("[🔴] 停止录制，退出遥操作")
                self.prev_teleop_btn = teleop_btn

                # ========== 蓝色按钮：录制启停 ==========
                if record_btn and not self.prev_record_btn:
                    if self.state == self.State.TELEOP:
                        # 开启录制
                        if self.dataset is None:
                            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                            out_path = Path(f"{self.OUTPUT_ROOT}_{stamp}")
                            self.dataset = LeRobotDataset.create(
                                repo_id=self.DATASET_NAME,
                                fps=self.FPS,
                                features=features,
                                root=str(out_path),
                                robot_type=self.ROBOT_TYPE,
                                use_videos=self.has_video,
                                image_writer_processes=4,
                                image_writer_threads=4,
                                streaming_encoding=self.has_video,
                            )
                            print(f"\n数据集创建路径：{out_path}")
                        self.state = self.State.RECORDING
                        self._is_recording = True
                        self._record_last_tick = time.perf_counter()
                        self.frame_count = 0
                        self.add_frame_times.clear()
                        self.t0 = now
                        print(f"[🔵] 开始录制 Episode {self.episode_idx}")

                    elif self.state == self.State.RECORDING:
                        # 关闭录制
                        self._is_recording = False
                        if self.frame_count > 0:
                            self.dataset.save_episode()
                            elapsed = now - self.t0
                            fps_real = self.frame_count / elapsed if elapsed > 0 else 0
                            print(f"\n[■] Episode {self.episode_idx} 保存 | 帧数:{self.frame_count} 实际fps:{fps_real:.1f}")
                            self.episode_idx += 1
                        self.frame_count = 0
                        self.state = self.State.TELEOP
                        print("[🔵] 停止录制，保持遥操作")
                self.prev_record_btn = record_btn

                # ========== 遥操作运动下发 ==========
                if self.state in (self.State.TELEOP, self.State.RECORDING):
                    action = exo_data["action"]
                    left_target = action[:7].tolist()
                    right_target = action[7:].tolist()
                    self.robot_ctrl.set_joint_target(left_target, right_target, duration=2.0, frequency=100)

                # ========== 性能打印 ==========
                if self.state == self.State.RECORDING and now - self.last_report >= 5 and self.add_frame_times:
                    avg_t = sum(self.add_frame_times) / len(self.add_frame_times)
                    print(f"[性能] add_frame平均耗时 {avg_t:.2f}ms")
                    self.add_frame_times.clear()
                    self.last_report = now

                if self.state in (self.State.TELEOP, self.State.RECORDING):
                    joystick = exo_data["joystick"]
                    v,w = self.joystick_to_vel(joystick[0], joystick[1])
                    self.robot_ctrl.set_speed(v, w)
                time.sleep(1.0 / self.STREAM_FREQ)

        except KeyboardInterrupt:
            print("\n\n[用户中断，准备退出]")
            # 关闭录制
            self._is_recording = False
            if self.state == self.State.RECORDING and self.frame_count > 0:
                self.dataset.save_episode()
                print("保存当前录制episode完成")
            if self.dataset is not None:
                print("finalize数据集...")
                self.dataset.finalize()
        finally:
            self.stop_recorder_thread()
            self.release_hardware()
            if self.dataset:
                print(f"\n数据集根目录：{self.dataset.root}")
                print(f"总episode数量：{self.dataset.num_episodes}")
            print("程序正常退出")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="外骨骼遥操作与 LeRobot 数据录制")
    parser.add_argument("--config", default="config/config.yaml")
    args = parser.parse_args()
    recorder = ExoTeleopRecorder(args.config)
    recorder.run()
