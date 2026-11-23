from robot_control_gr3 import RobotController
from robot_base import BaseRobotController
import numpy as np

class Gr3RobotController(BaseRobotController):
    def __init__(self, robot_cfg: dict):
        # 从yaml读取该机器人专属参数
        self.cfg = robot_cfg
        self.controller = RobotController(
            # ip=self.cfg["ip"],
            # port=self.cfg["port"]
            # 其他xoip需要的参数
        )

    def open(self, speed) -> bool:
        return self.controller.open()

    def close(self):
        self.controller.close()

    def set_joint_target(self, left_target: np.ndarray, right_target: np.ndarray, duration, frequency):
        # 适配内部调用格式，比如检查维度、clip限位
        # print(f"left_target: {left_target}")
        print(f"right_target: {right_target}")
        self.controller.send_joints(left_target, right_target, duration, frequency)

    def get_joint_state(self) -> np.ndarray | None:
        state = self.controller.read(has_hand = False)
        if state is None:
            return None
        return np.array(state, dtype=np.float32)
