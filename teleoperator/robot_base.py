from abc import ABC, abstractmethod
import numpy as np

class BaseRobotController(ABC):
    requires_home = True
    alignment_moves_robot = True

    @abstractmethod
    def open(self,speed) -> bool:
        """启动连接"""
        ...

    @abstractmethod
    def close(self):
        """关闭连接"""
        ...

    @abstractmethod
    def set_joint_target(self, left_target: np.ndarray, right_target: np.ndarray, duration, frequency):
        """下发关节目标角度（弧度）"""
        ...

    @abstractmethod
    def get_joint_state(self) -> np.ndarray | None:
        """读取当前机器人真实关节角度，None=无数据"""
        ...

    def align_for_teleop(self, left_target, right_target, duration, frequency):
        """进入遥操作前对齐；旧机器人默认执行原有绝对关节运动。"""
        self.set_joint_target(left_target, right_target, duration, frequency)

    def action_for_dataset(self, exo_action: np.ndarray) -> np.ndarray:
        """返回实际命令空间中的动作，默认保持旧版双臂 14 维格式。"""
        return np.asarray(exo_action, dtype=np.float32)

    def stop_motion(self, reason: str = "operator_requested"):
        """停止当前遥操作任务；不支持停止接口的旧控制器保持原行为。"""
        return None

    def tick(self):
        """周期安全检查钩子。"""
        return None

    def set_speed(self, v, w):
        print("没有速度控制")
