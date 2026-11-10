# CVTE D3 常用 API 与控制方式

更新日期：2026-11-10

适用范围：CVTE D3 双臂机器人、官方 MuJoCo 仿真，以及本项目第一阶段右臂-only 实验。

## 1. 先选对接口层

本项目存在三层接口，它们的名称相似，但安全语义和返回值不同。

| 接口层 | 推荐用途 | 本项目结论 |
| --- | --- | --- |
| `src.simulation.D3RightArmSim` | 当前无真机阶段的右臂练习、单元测试 | 当前首选；只暴露右臂动作，左臂在仿真中冻结 |
| `carm_mujoco.CArmDualBot` | 官方 D3 模型的底层仿真、双臂功能研究 | 可作为底层参考；官方演示会操作双臂，不能直接作为本项目入口 |
| `carm.CArmDualBot` | 真机 SDK | 真机到场后使用；必须先完成安全包装和 G0–G2 检查 |
| LeRobot `D3Robot` | 数据采集与策略动作适配 | 当前版本存在构造时准备和双臂硬编码复位等阻断项，不能原样上机 |

推荐调用关系：

```text
练习/单元测试
    └── D3RightArmSim
          └── 官方 MuJoCo CArmDualBot

真机采集/推理（待实现）
    └── 项目右臂安全包装
          └── carm.CArmDualBot
```

## 2. 数据约定

| 数据 | 结构 | 当前项目约定 |
| --- | --- | --- |
| 右臂关节 | `[q1, q2, q3, q4, q5, q6, q7]` | 7 维；仿真使用弧度 `rad` |
| 法兰位姿 | `[x, y, z, qx, qy, qz, qw]` | 位置使用米 `m`；四元数顺序为 `xyzw` |
| 关节速度 | 7 维 | 仿真使用 `rad/s` |
| 关节力矩 | 7 维 | 仿真使用 `N·m` |
| 末端外力 | `[Fx, Fy, Fz, Mx, My, Mz]` | 力与力矩六维旋量 |
| 夹爪宽度 | 标量 | 官方 MuJoCo 表示两指尖间距，单位 `m` |

真机的关节顺序、单位、零位、法兰坐标系、工具坐标系和夹爪量程必须从交付版本的 `get_*_config()` 与实测结果确认，不能只依据仿真数值。

## 3. 控制方式怎么选

### 3.1 关节控制与位姿控制

| 方式 | 输入 | 优点 | 局限 | 当前建议 |
| --- | --- | --- | --- | --- |
| 关节控制 | 7 个关节角 | 契约直接、容易做逐关节限位与回放 | 末端路径不一定是直线 | 第一批采集和策略基线优先 |
| 位姿控制 | `xyz + xyzw` | 符合操作任务直觉，便于做空间增量 | 依赖 IK、坐标标定和奇异位形处理 | 标定稳定后作为对照 |
| `both` | 同时记录 joint 与 pose | 诊断信息丰富 | 当前 LeRobot 驱动不支持 `both` 推理，动作语义易混淆 | 仅诊断采集，不作为首个推理契约 |

### 3.2 规划运动、跟随运动与轨迹运动

| 类型 | 典型 API | 行为 | 适用场景 |
| --- | --- | --- | --- |
| 点到点规划 | `move_right_joint()`、`move_right_pose()` | 内部生成轨迹并到达单个目标 | 回零、到达准备位、低频调试 |
| 高频跟随 | `track_right_joint()`、`track_right_pose()` | 连续接收目标，接近透传/伺服式跟随 | 遥操作和 VLA 策略循环 |
| 多点轨迹 | `move_right_joint_traj()`、`move_right_pose_traj()` | 按目标序列和时间戳执行 | 固定脚本、轨迹回放 |
| 运动学查询 | `forward_kine_right()`、`inverse_kine_right()` | 只计算 FK/IK，不应产生运动 | 标定、目标检查、离线验证 |

使用原则：策略循环使用 `track_*` 时，项目安全层必须额外负责命令频率、陈旧命令超时、速度/加速度限制、NaN/Inf 拒绝和断连停止。不要把 `move_*` 的内部规划能力误认为策略安全层。

### 3.3 控制模式

锁定版本的官方 MuJoCo 和 C++ 示例给出以下模式：

| 模式 | 名称 | 含义 | 当前用途 |
| --- | --- | --- | --- |
| `0` | IDLE | 空闲，不接受常规运动目标 | 初始/停止状态 |
| `1` | POSITION / PV | 位置控制 | 当前仿真练习默认模式 |
| `2` | MIT | 位置、速度与力矩相关控制 | 仅在参数与安全边界明确后测试 |
| `3` | DRAG / CURRENT | 拖动/电流相关模式 | 示教或拖动场景，真机需现场确认 |
| `4` | PF | 力位混合或力矩限制 | 高风险高级模式，首阶段不使用 |

注意：厂商 ROS 话题表仍写“模式 0–3”，与当前 C++ 示例的 0–4 不一致。真机到场后应查询控制器版本，并通过只读状态和厂商文档确认模式编号；不得根据仿真实现直接切换真机模式。

## 4. 项目仿真 API：`D3RightArmSim`

这是当前最推荐的练习入口。它对关节维度、有限值和官方模型关节限位进行检查，并在每个仿真步冻结左臂。

### 4.1 创建与关闭

```python
from src.simulation import D3RightArmSim

with D3RightArmSim(render=True) as sim:
    print(sim.timestep_s)
    print(sim.right_joint_limits_rad)
```

| API | 返回 | 说明 |
| --- | --- | --- |
| `D3RightArmSim(render=False, realtime=None)` | 仿真实例 | `render=True` 打开 MuJoCo 原生窗口 |
| `close()` | `None` | 关闭 viewer；优先使用 `with` 自动关闭 |
| `step(steps=1)` | `None` | 推进指定仿真步数 |
| `timestep_s` | `float` | 当前官方模型为 `0.001 s` |
| `right_joint_limits_rad` | 7 组上下限 | 官方 MJCF 中的右臂关节限位 |
| `left_joint_position_rad` | 7 维只读状态 | 用于验证左臂保持冻结，不能发送左臂动作 |

### 4.2 状态读取

```python
state = sim.state()
print(state.joint_position_rad)
print(state.joint_velocity_rad_s)
print(state.joint_torque_nm)
print(state.flange_pose_xyz_quat)
print(state.gripper_width_m)
```

`state()` 返回不可变的 `RightArmState` 快照，包含右臂关节、法兰、夹爪和仿真时间。

### 4.3 常用运动

```python
target = [0.25, -0.45, 0.20, 0.55, 0.0, 0.20, 0.0]

state = sim.move_joint(target, duration_s=1.5, settle_s=0.2)
pose = sim.forward_kinematics(target)
solution = sim.inverse_kinematics(pose, reference_joint_rad=target)
state = sim.move_pose(pose, duration_s=1.5)
state = sim.set_gripper(0.065, force_limit_n=5.0, duration_s=0.8)
```

| API | 输入 | 返回/失败方式 |
| --- | --- | --- |
| `move_joint(target_rad, duration_s, settle_s)` | 7 维关节目标 | 返回 `RightArmState`；非法维度、NaN/Inf、越限会抛异常 |
| `move_pose(pose_xyz_quat, duration_s, settle_s)` | 7 维法兰位姿 | 返回状态；IK 不收敛会抛异常 |
| `set_gripper(width_m, force_limit_n, duration_s)` | 目标宽度和力限制 | 返回状态；目标非法会抛异常 |
| `forward_kinematics(joint_position_rad)` | 7 维关节角 | 返回 `xyz + xyzw` |
| `inverse_kinematics(pose_xyz_quat, reference_joint_rad)` | 位姿与参考关节角 | 返回 7 维数值 IK 解；不收敛会抛异常 |

## 5. 官方 MuJoCo 底层 API

官方接口适合查阅 D3 的原始仿真能力。返回值通常为 `1` 成功、`0` 失败，与真机 SDK 的失败返回值不同。

```python
from carm_mujoco import CArmDualBot

robot = CArmDualBot("models/carm_d3_mjcf/carm_d3.xml", render=True)
robot.set_control_mode(1)
robot.move_right_joint([0.0] * 7, desire_time=1.5, is_sync=True)
robot.close()
```

常用右臂 API：

| 类别 | API |
| --- | --- |
| 仿真推进 | `step(nstep=1)`、`close()` |
| 模式 | `set_control_mode(mode)` |
| 实际关节 | `get_right_joint_pos()`、`get_right_joint_vel()`、`get_right_joint_tau()` |
| 规划关节 | `get_right_plan_joint_pos()`、`get_right_plan_joint_vel()`、`get_right_plan_joint_tau()` |
| 法兰状态 | `get_right_cart_pose()`、`get_right_plan_cart_pose()` |
| 外力 | `get_right_joint_external_tau()`、`get_right_cart_external_force()` |
| 高频跟随 | `track_right_joint(targets, gripper_pos=-1)`、`track_right_pose(targets, gripper_pos=-1)` |
| 点到点 | `move_right_joint(target, desire_time=-1, is_sync=True)`、`move_right_pose(...)` |
| 直线/轨迹 | `move_right_line_joint()`、`move_right_line_pose()`、`move_right_joint_traj()`、`move_right_pose_traj()` |
| 运动学 | `forward_kine_right()`、`inverse_kine_right()` 及对应批量接口 |
| 夹爪 | `set_right_gripper(pos, tau=10)`、`get_right_gripper_pos/vel/tau()` |

`is_sync=True` 表示调用阻塞到该段运动结束；`False` 表示提交目标后立即返回，需要持续调用 `step()` 推进仿真。`desire_time > 0` 使用指定时长的轨迹；官方 MuJoCo 中 `desire_time == -1` 使用时间最优双 S 规划。

本项目不直接使用官方 `test_dual_api.py` 作为练习入口，因为其中多个命令同时运动左右臂。

## 6. 真机 `carm.CArmDualBot` 常用 API

### 6.1 连接与基础控制

| API | 作用 | 风险提示 |
| --- | --- | --- |
| `CArmDualBot(address, port=8090, timeout=...)` | 创建双臂控制对象 | 地址和构造副作用必须在交付版本实测 |
| `connect(address)` / `disconnect()` | 连接/断开控制器 | 断开前应先停止任务并确认安全保持 |
| `is_connected()` | 查询连接状态 | 还需分别检查左右臂状态 |
| `get_version()` | 读取控制器/SDK 版本 | 应写入实验日志 |
| `get_left_status()` / `get_right_status()` | 读取连接、伺服、FSM 等状态 | 首次联调从这些只读接口开始 |
| `set_ready()` | 控制器进入准备状态 | 不是“绝对安全的复位”；现场验证后才能调用 |
| `set_servo_enable(flag)` | 伺服使能/失能 | 失能可能导致机械臂下坠，不可当作普通停止按钮 |
| `set_control_mode(mode)` | 切换控制模式 | 模式编号需与交付固件核对 |
| `task_stop()` | 停止当前任务 | 上机前需独立验证 |
| `emergency_stop()` | 软件急停 | 不能替代可触达的硬件急停 |
| `set_speed_level(level, response_level=20)` | 设置速度等级与变化速度 | 首次运动使用最低经确认等级 |
| `set_collision_config(enable, sensitivity)` | 配置碰撞检测 | 厂商示例称灵敏度 `0–3`，`0` 最灵敏；真机复核 |

厂商示例中的命令类方法通常返回 `1` 成功、`-1` 失败；查询方法直接返回数据。所有返回值和异常都必须记录，不能忽略失败后继续发送动作。

### 6.2 状态与配置

| 目的 | 右臂 API |
| --- | --- |
| 配置与限位 | `get_right_config()`、`get_right_eeff_config()` |
| 机械臂状态 | `get_right_status()` |
| 实际关节状态 | `get_right_joint_pos/vel/tau()` |
| 实际法兰位姿 | `get_right_cart_pose()` |
| 规划目标 | `get_right_plan_joint_pos/vel/tau/cart_pose()` |
| 外力估计 | `get_right_joint_external_tau()`、`get_right_cart_external_force()` |
| 工具坐标 | `set_right_tool_index()`、`get_right_tool_index()`、`get_right_tool_coordinate()` |

第一轮真机连接只允许执行这一类只读查询。左臂可以读取诊断状态，但不能出现在动作发送路径中。

### 6.3 运动与末端执行器

| 目的 | 真机右臂 API | 当前建议 |
| --- | --- | --- |
| 高频关节跟随 | `track_right_joint(targets, eeff_pos=-1)` | 遥操作/VLA 的候选主接口 |
| 高频位姿跟随 | `track_right_pose(targets, eeff_pos=-1)` | 标定与 IK 风险验证后使用 |
| 点到点关节 | `move_right_joint(target, desire_time=-1, is_sync=True)` | 准备位和低速验证 |
| 点到点位姿 | `move_right_pose(target, desire_time=-1, is_sync=True)` | 坐标系验证后使用 |
| 多点关节轨迹 | `move_right_joint_traj(points, eeff_pos, stamps, is_sync)` | 固定轨迹回放 |
| 多点位姿轨迹 | `move_right_pose_traj(points, eeff_pos, stamps, is_sync)` | 高阶功能，首阶段非必需 |
| 通用末端控制 | `set_right_eeff(pos, vel, tau)` | 新接口优先 |
| 旧夹爪接口 | `set_right_gripper(...)` | 厂商已标记 deprecated；当前 LeRobot 适配仍在使用，需迁移验证 |
| 运动学 | `forward_kine_right(...)`、`inverse_kine_right(...)` | 接口参数次序与 MuJoCo 版本不同，不能直接互换代码 |

真机双臂 SDK 的 `move_right_line_joint/pose` 已从当前 C++ 头文件移除，而官方 MuJoCo 仍实现同名直线接口。不要因为仿真中存在就假定真机可用。

## 7. ROS 1 / ROS 2 常用话题

厂商 ROS 节点把公共控制和左右臂动作分开。当前实验只允许使用右臂动作话题。

| 类型 | 右臂/公共话题 | 对应 SDK |
| --- | --- | --- |
| 状态 | `right_real_joint_state` | `get_right_joint_*()` |
| 法兰位姿 | `right_flange_cart_state` | `get_right_cart_pose()` |
| 机械臂状态 | `right_arm_state` | `get_right_status()` |
| 点到点关节 | `right_move_joint` | `move_right_joint()` |
| 点到点位姿 | `right_move_pose` | `move_right_pose()` |
| 关节跟随 | `right_move_tracking_joint` | `track_right_joint()` |
| 位姿跟随 | `right_move_tracking_pose` | `track_right_pose()` |
| 末端控制 | `right_set_eeff` | `set_right_eeff()` |
| 公共控制 | `ready`、`emergency_stop`、`set_servo_enable`、`set_speed_level`、`set_control_mode` | 对应同名控制 API |
| 事件 | `task_completion`、`carm_error` | 任务完成与错误回调 |

ROS 节点示例默认可能调用 `set_ready()`，且示例地址不能直接用于现场。启动前必须审查初始化路径，并通过命名空间或代码级白名单禁止全部 `left_*` 动作话题。

## 8. LeRobot D3 动作契约

锁定版本的 `D3Robot` 支持三种 `move_mode`：

| `move_mode` | 状态/动作字段 | 发送接口 |
| --- | --- | --- |
| `joint` | `joint0_right` … `joint6_right`，可选 `gripper_right` | `track_right_joint()` |
| `pose` | `tx_right, ty_right, tz_right, qx_right, qy_right, qz_right, qw_right`，可选夹爪 | `track_right_pose()` |
| `both` | 同时包含 joint 与 pose | 采集时只用 joint 驱动；推理明确不支持 |

当前驱动即使 `left_arm.enable_action=false`，特征 schema 仍包含左臂字段；构造函数还会调用 `set_ready()` 和双臂硬编码复位。因此它只能作为接口参考，不能在完成项目右臂-only schema、安全连接和停止逻辑之前直接连接真机。

## 9. 仿真到真机的对应关系

| 当前仿真练习 | 将来的真机语义 | 迁移前必须补齐 |
| --- | --- | --- |
| `sim.state()` | `get_right_*()` 状态集合 | 时间戳、错误码、连接状态 |
| `sim.move_joint()` | `move_right_joint()` | 真机限位、速度、加速度、急停 |
| `sim.move_pose()` | `move_right_pose()` | 基座/法兰/工具坐标标定与 IK |
| `sim.set_gripper()` | `set_right_eeff()` 或过渡期夹爪接口 | 实际量程、力限制、物体接触测试 |
| `sim.forward_kinematics()` | `forward_kine_right()` | 参数次序和工具号适配 |
| `sim.inverse_kinematics()` | `inverse_kine_right()` | 多解、奇异位形、参考关节与失败处理 |

不要把仿真中的 HOME、练习姿态、夹爪宽度或控制增益直接复制到真机。可迁移的是动作契约、校验逻辑和测试顺序，不是未经验证的数值。

## 10. 推荐学习与联调顺序

1. 在 MuJoCo 中使用 `state()`、`move_joint()` 和 FK 熟悉 7 个关节及单位。
2. 使用 IK 观察同一末端位姿的多解与参考关节依赖。
3. 比较点到点 `move_*` 与高频 `track_*` 的语义，不在真机前假定二者安全行为相同。
4. 真机到场后先执行版本、配置、状态和限位的只读采集。
5. 完成硬件急停、软件停止、右臂-only 和最低速度验证后，才做单关节小幅运动。
6. 最后才接入外骨骼连续跟随和 VLA 策略输出。

## 11. 资料基线

- `cvte-robotics/carm_demo`：`5e6e4a5bee16222194392c76a12836ba15cf5136`
- `cvte-robotics/carm-mujoco`：`0ea54f12ffe17b2f8f65eebbb51e6eab2b6af643`
- `cvte-robotics/carm-lerobot`：`31b1349bc1906765dd522324659a07bd7d281c10`
- 项目仿真包装：`src/simulation/d3_right_arm_sim.py`
- 真机安全门：`docs/operations/hardware-safety-checklist.md`

如交付 SDK、控制器固件或官方仓库版本变化，应重新核对本表，不要仅按函数同名判断行为兼容。
