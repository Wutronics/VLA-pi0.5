from robot_control_xoip import RobotController
from robot_base import BaseRobotController
import numpy as np
import os

class XoipRobotController(BaseRobotController):
    def __init__(self, robot_cfg: dict):
        # 从yaml读取该机器人专属参数
        self.cfg = robot_cfg
        local_env = self.cfg.get("local_address_env", "XOIP_LOCAL_ADDRESS")
        remote_env = self.cfg.get("address_env", "XOIP_CONTROLLER_ADDRESS")
        local_ip = os.environ.get(local_env)
        remote_ip = os.environ.get(remote_env)
        if not local_ip or not remote_ip:
            raise ValueError(f"请设置 {local_env} 和 {remote_env}")
        self.controller = RobotController(
            local_ip=local_ip,
            remote_ip=remote_ip,
            port=int(self.cfg.get("port", 8001)),
        )

    def open(self, speed) -> bool:
        return self.controller.open(speed)

    def close(self):
        self.controller.close()

    def set_joint_target(self, left_target: np.ndarray, right_target: np.ndarray, duration, frequency):
        # 适配内部调用格式，比如检查维度、clip限位
        # print(f"left_target: {left_target}")
        # print(f"right_target: {right_target}")
        self.controller.send_joints(left_target, right_target)
        # pass

    def get_joint_state(self) -> np.ndarray | None:
        state = self.controller.read()
        if state is None:
            return None
        return np.array(state, dtype=np.float32)

    def set_speed(self, v, w):
        CAN_PORT = 1
        forward = v >= 0
        turn_left = w >= 0
        self.controller.sendSpeedControl(
            port=CAN_PORT,
            forward=forward,
            linear_vel=abs(v),
            turn_left=turn_left,
            angular_vel=abs(w),
            light=0
        )
