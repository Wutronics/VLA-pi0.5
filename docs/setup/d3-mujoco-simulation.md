# CVTE D3 MuJoCo 仿真与入门测试

## 用途与边界

本环境使用 CVTE 官方 `carm-mujoco` 中的 D3 MJCF、网格和 `CArmDualBot` 接口。项目入口在其外增加了右臂-only 约束：练习代码只暴露右臂动作，左臂在初始位姿冻结。

它适合熟悉以下内容：

- D3 是每臂 7 自由度的双臂系统；
- 关节角、关节限位、法兰位姿和夹爪宽度的单位；
- 关节空间运动、正运动学、数值逆运动学和夹爪控制；
- 本项目第一阶段“只控制右臂”的动作契约。

它不能替代真机上的碰撞、线缆、控制延迟、标定误差、急停、负载和夹持力验证。仿真通过后仍必须从 G0/G1 真机安全门开始。

机械臂接口、控制模式、仿真/真机差异和 ROS/LeRobot 映射统一整理在 [D3 常用 API 与控制方式](../operations/d3-common-api-and-control.md)。

## 1. 安装

在仓库根目录执行：

```bash
bash scripts/simulation/setup_simulation.sh
```

脚本会下载锁定版本的官方模型，并创建独立的 `.venv-sim`。当前锁定版本为 MuJoCo 3.2.3、NumPy 1.24.4 和 `carm-mujoco` 提交 `0ea54f1`。

## 2. 无界面自检

```bash
bash scripts/simulation/run_d3_sim.sh --demo all
.venv-sim/bin/python -m unittest tests.unit.test_d3_right_arm_sim -v
```

预期最后分别出现：

```text
PASS: selected simulation exercises completed.
OK
```

自动测试会验证模型契约、非法动作拒绝、右臂关节跟踪、左臂零漂移以及 FK/IK 往返。

## 3. 打开可视化窗口

本地桌面或启用图形转发后运行：

```bash
bash scripts/simulation/run_d3_sim.sh --demo all --render
```

窗口中应看到 D3 右臂移动到练习姿态后回零、右夹爪打开再闭合；左臂保持不动。

## 4. 分项练习

先只读状态：

```bash
bash scripts/simulation/run_d3_sim.sh --demo state
```

练习 7 关节目标与回零：

```bash
bash scripts/simulation/run_d3_sim.sh --demo joint --render
```

练习夹爪宽度控制（单位为米）：

```bash
bash scripts/simulation/run_d3_sim.sh --demo gripper --render
```

练习关节角与 `[x, y, z, qx, qy, qz, qw]` 法兰位姿之间的转换：

```bash
bash scripts/simulation/run_d3_sim.sh --demo kinematics
```

## 5. 编程接口最小示例

```python
from src.simulation import D3RightArmSim

with D3RightArmSim(render=True) as sim:
    state = sim.move_joint(
        [0.25, -0.45, 0.20, 0.55, 0.0, 0.20, 0.0],
        duration_s=1.5,
    )
    print(state.joint_position_rad)
    print(state.flange_pose_xyz_quat)
```

包装器会检查动作必须是 7 维有限数值且位于官方模型关节限位内。不要绕过包装器直接调用官方 `test_dual_api.py` 作为本项目练习入口，因为该脚本的多个命令会同时运动左右臂。

## 6. 建议练习顺序

1. 读懂初始关节、法兰位姿和单位。
2. 每次只改变一个右臂关节，观察末端位置变化。
3. 使用 FK 预测目标姿态，再用 IK 求回关节角。
4. 练习夹爪宽度，记录目标值与实际值的稳态误差。
5. 真机到场后，只复用动作契约和测试思路；不要直接复用仿真目标值。
