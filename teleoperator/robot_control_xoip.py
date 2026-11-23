# -*- coding: utf-8 -*-
import math
import threading
import time
from typing import Iterable, List, Optional
import numpy as np
import xoip

def _on_move_end(robot_id, _end):
    print("机器人回调", robot_id)

class RobotController:
    """HCXSDK 机械臂简单封装。"""
    def __init__(self, local_ip: str, remote_ip: str, port: int = 8001):
        """
        :param local_ip:  本机 IP，与下位机同网段
        :param remote_ip: 下位机 IP
        :param port:      通信端口
        """
        xoip.RobotManager.init_data(local_ip, remote_ip, port)
        self._move_end = threading.Event()
        self._move_robot_id: Optional[int] = None
        # 回调函数签名由 SDK 固定，这里闭包内只更新事件

        self._on_move_end = _on_move_end
        xoip.RobotManager.registerMoveJointsEndCb(self._on_move_end)
        self.open(10)

        # 流式控制 (后台线程持续下发, 主线程不阻塞)
        self._streaming = False
        self._stream_thread: Optional[threading.Thread] = None
        self._stream_freq = 100.0
        self._max_velocity = 30.0   # 关节最大角速度 (度/秒)
        self._target_lock = threading.Lock()
        self._latest_targets: dict[int, List[float]] = {}   # robot_id -> 最新目标
        self._sent_targets: dict[int, List[float]] = {}     # robot_id -> 上一次发送值

    # ---------- 基础控制 ----------
    def enable(self, on: bool = True) -> bool:
        """上/下使能。"""
        return xoip.RobotManager.robotEnableOrNot(on)

    def is_enabled(self) -> bool:
        return xoip.RobotManager.getRobotEnableStatus()

    def set_global_speed(self, percent: float) -> bool:
        """全局速度比例，单位 %。"""
        return xoip.RobotManager.setGlobalSpeed(percent)

    def close(self) -> bool:
        print(f"robot close...")
        # return self.enable(False)

    # ---------- 运动 ----------
    def move_joints(self, robot_id: int, joints: Iterable[float], wait: bool = False) -> bool:
        """
        关节运动。
        :param wait:   是否等待到位（结合回调 + 位置闭环）
        :param tol:    到位判定容差
        :param timeout: 等待超时时间（秒），None 表示不限
        """
        target = list(joints)
        self._move_end.clear()
        self._move_robot_id = robot_id
        ok = xoip.RobotManager.moveJoints(robot_id, target, wait)
        return True

    def get_joints(self, robot_id: int, feedback: bool = True) -> List[float]:
        return list(xoip.RobotManager.getJoints(robot_id, feedback))

    def read(self):
        """读取机器人全部关节角，返回 ndarray[26]
        (左臂7 + 右臂7
        如果机器人没有手部关节组，手部部分返回全零。
        """
        try:
            left = self.get_joints(1)
            right = self.get_joints(2)
        except Exception:
            left, right = [0]*7, [0]*7

        joints = np.array(
            list(left) + list(right),dtype=np.float32,
        )

        return joints

    def send_joints(self, left: list, right: list, duration: float = 0.05, frequency: float = 100.0):
        """
        同时设置左右手目标关节（非阻塞，依赖流式后台平滑插值）
        robot_id=1 → 左手 left
        robot_id=2 → 右手 right
        :param left: 左手目标关节角列表
        :param right: 右手目标关节角列表
        :param duration: 过渡时间（秒），预留参数，当前流式依靠_max_velocity限速平滑
        """
        # 方案A：使用已有的流式目标更新（推荐，平滑连续）
        import math
        # 弧度转角度
        left_deg = [math.degrees(r) for r in left]
        # left_deg = self.get_joints(1)
        # index = 6
        # left_deg[index] = math.degrees(left[index])
        # print(f"left_deg is {left_deg}")
        right_deg = [math.degrees(r) for r in right]
        # right_deg = self.get_joints(2)
        # index = 6
        # right_deg[index] = math.degrees(right[index])
        # print(f"left_deg is {left_deg}, right_deg is {right_deg}")
        # print(f"left_deg is {left_deg}")
        # print(f"right_deg is {right_deg}")
        self.move_joints(1, left_deg, wait=True)
        self.move_joints(2, right_deg, wait=True)


    # ---------- 末端可达性 ----------
    def try_worlds(self, robot_id: int, worlds: Iterable[float]):
        """返回 (ok, thejoints, pmask, nmask)。"""
        return xoip.RobotManager.tryWorlds(robot_id, list(worlds))

    # ═══════════════════════════════════════════════════════
    # 流式模式: 后台线程持续下发, 主线程不阻塞
    # ═══════════════════════════════════════════════════════
    def start_streaming(self, frequency: float = 100):
        """启动后台线程, 以固定频率向已注册 robot_id 持续下发最新目标."""
        if self._streaming:
            return
        self._stream_freq = frequency
        self._streaming = True
        self._stream_thread = threading.Thread(
            target=self._streaming_loop,
            daemon=True,
        )
        self._stream_thread.start()
        print(f"[Robot] 流式控制已启动 ({frequency}Hz)")

    def stop_streaming(self):
        """停止后台线程."""
        self._streaming = False
        if self._stream_thread is not None:
            self._stream_thread.join(timeout=2.0)
            self._stream_thread = None
            print("[Robot] 流式控制已停止")

    def set_target(self, robot_id: int, joints: Iterable[float]):
        """注册/更新指定 robot_id 的目标关节角 (非阻塞, 仅更新内部变量)."""
        with self._target_lock:
            self._latest_targets[robot_id] = list(joints)

    def _streaming_loop(self):
        """后台线程: 持续以固定频率下发各 robot 的最新目标, 带每关节速度限制."""
        period = 1.0 / self._stream_freq
        while self._streaming:
            with self._target_lock:
                targets = {k: list(v) for k, v in self._latest_targets.items()}

            for robot_id, target in targets.items():
                try:
                    # 首次发送: 从机器人当前位置起始
                    if robot_id not in self._sent_targets:
                        self._sent_targets[robot_id] = list(
                            xoip.RobotManager.getJoints(robot_id, True)
                        )

                    # 每步最大关节变化量
                    max_delta = self._max_velocity / self._stream_freq
                    sent = self._sent_targets[robot_id]
                    n = min(len(sent), len(target))
                    for i in range(n):
                        lo = sent[i] - max_delta
                        hi = sent[i] + max_delta
                        sent[i] = max(lo, min(hi, target[i]))
                    print(f"robot_id={robot_id}, sent={sent}")
                    # xoip.RobotManager.moveJoints(robot_id, sent, False)
                except Exception as e:
                    print(f"[Robot] 下发异常 robot_id={robot_id}: {e}")

            time.sleep(period)

    # ---------- 内部工具 ----------
    def _wait_reached(self, robot_id: int, target: List[float], tol: float,
                      timeout: Optional[float]) -> bool:
        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            cur = self.get_joints(robot_id, True)
            n = min(len(cur), len(target))
            if all(math.isclose(target[i], cur[i], rel_tol=tol) for i in range(n)):
                return True
            if deadline is not None and time.monotonic() > deadline:
                return False
            time.sleep(0.1)

    def canReceiveCallback(msg):
        print("CAN接收数据:", msg)

    def open(self, speed = 10):
        if not xoip.RobotManager.detachHMI():
            print("detachHMI error.")
            time.sleep(1)

        if not xoip.RobotManager.getDetachHMIStatus():
            print("detachHMI error.")
        time.sleep(1)
        print("detachHMI success.")\

        print("连接成功，上使能，全局速度：", speed)
        self.set_global_speed(speed)
        time.sleep(1)
        # clear alarm
        alarm_list = []
        while xoip.RobotManager.isAlarming(alarm_list):
            for a in alarm_list:
                print(a)
            time.sleep(1)
            xoip.RobotManager.clearAlarm()
        if not self.enable(True):
            print("上使能失败")
            return
        time.sleep(1)
        if not self.is_enabled():
            print("机器人没有上使能")
            return
        xoip.IOManager.registerCanReadCb(self.canReceiveCallback)
        CAN_PORT = 1
        CAN_BAUD = 500000
        if not xoip.IOManager.openCAN(CAN_PORT, CAN_BAUD):
            print(f"open can {CAN_PORT} failed..")
            return

    def calculateCRC8(self, data: bytes) -> int:
        crc = 0x00
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x01:
                    crc = (crc >> 1) ^ 0x8C
                else:
                    crc >>= 1
        return crc & 0xFF

    def sendSpeedControl(self, port: int, forward: bool, linear_vel: float, turn_left: bool, angular_vel: float, light: int = 0) -> bool:
        """
        对应C++ sendSpeedControl
        :param port: CAN端口号
        :param forward: True前进，False后退
        :param linear_vel: 线速度 m/s (传入绝对值)
        :param turn_left: True左转，False右转
        :param angular_vel: 角速度 rad/s (传入绝对值)
        :param light: 车灯参数 0~15
        :return: 发送成功返回True
        """
        data = bytearray([0] * 8)

        # Byte0 前进后退标志
        data[0] = 0x01 if forward else 0x00

        # Byte1 Byte2 线速度 cm/s
        linear_cm_s = int(linear_vel * 100)
        if linear_cm_s > 125:
            linear_cm_s = 125
        data[1] = (linear_cm_s >> 8) & 0xFF
        data[2] = linear_cm_s & 0xFF

        # Byte3 转向标志
        data[3] = 0x01 if turn_left else 0x00

        # Byte4 Byte5 角速度 0.01rad/s
        angular_001rad_s = int(angular_vel * 100)
        if angular_001rad_s > 105:
            angular_001rad_s = 105
        data[4] = (angular_001rad_s >> 8) & 0xFF
        data[5] = angular_001rad_s & 0xFF

        # Byte6 车灯 bit1~bit4
        data[6] = (light & 0x0F) << 1

        # Byte7 CRC8 前7字节校验
        crc_val = self.calculateCRC8(bytes(data[:7]))
        data[7] = crc_val & 0xFF

        # 打印报文，和C调试输出对齐
        hex_str = ", ".join(f"0x{x:02x}" for x in data)
        print(f"CAN报文: {hex_str}")

        # CAN发送接口：xoip.IOManager.writeCAN
        ret = xoip.IOManager.writeCAN(port, 0x200, bytes(data), len(data))

        if ret:
            print("[发送] 速度控制指令成功！CAN ID: 0x200")
            print(f"  方向: {'前进' if forward else '后退'}  线速度: {linear_vel:.3f} m/s")
            print(f"  转向: {'左转' if turn_left else '右转'}  角速度: {angular_vel:.3f} rad/s\n")
        else:
            print("[错误] 发送速度控制指令失败！\n")
        return ret

def main():
    import os
    local_ip = os.environ.get("XOIP_LOCAL_ADDRESS")
    remote_ip = os.environ.get("XOIP_CONTROLLER_ADDRESS")
    if not local_ip or not remote_ip:
        raise SystemExit("Set XOIP_LOCAL_ADDRESS and XOIP_CONTROLLER_ADDRESS first.")
    robot = RobotController(local_ip=local_ip, remote_ip=remote_ip, port=8001)

    robot.open(10)

    robot_id = 1

    joints = [0, 0, 0, 0, 0.5, 0, 0]

    # 第一次：不等待
    left = joints
    right = joints
    robot.send_joints(left, right, duration=0.05)

    # 第二次：等待到位

    # robot.send_joints(left, right, duration=0.05)

    cur = robot.get_joints(robot_id)
    print("当前关节：", cur)
    print("所有轴已经运动到位")

    time.sleep(10)
    print("下使能")
    robot.close()


if __name__ == "__main__":
    main()
