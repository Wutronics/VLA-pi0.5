# D3 基准测试报告（软件与仿真）

日期：2026-11-23

代码基线：`8e9a3c5`（执行测试前的 `main`）

范围：CVTE D3 右臂-only 仿真、安全契约和真机只读测试准备。

## 结论

软件/仿真基准、真机只读基准和首个真机运动基准均已通过。初次只读快照曾发现右臂 J6 反馈超出软限位；现场人工归零后，最新快照的 7 个右臂关节均回到限位内并通过运动预检。真机末端执行器、相机、外骨骼和连续策略基准仍未执行；每次动作前都必须重新运行只读预检。没有测试脚本会在未使用 `--execute` 的情况下连接或发送真机命令。

## 已完成的基准

| 基准 | 命令 | 结果 | 关键结果 |
| --- | --- | --- | --- |
| 依赖锁定 | `.venv-sim/bin/python` 导入检查 | 通过 | MuJoCo `3.2.3`，NumPy `1.24.4` |
| 右臂点到点运动 | `bash scripts/simulation/run_d3_sim.sh --demo all` | 通过 | 最大右臂关节误差 `0.00361 rad`；左臂漂移 `0` |
| 夹爪范围演示 | 同上 | 通过 | 开口 `0.0700 m`，闭合 `0.0181 m` |
| FK/IK 往返 | 同上 | 通过 | 法兰位置重建误差 `0.000000 m` |
| 软件单元测试 | `python -m unittest discover -s tests/unit -v` | 通过 | 16/16：模型契约、右臂隔离、FK/IK、只读约束、关节步进和安全适配器门禁 |
| 真机 SDK 只读快照 | `d3_hardware_readonly_benchmark.py --execute` | 通过 | SDK `1.0.260819`；左右臂均为 7 DOF；关节位置/速度/力矩和法兰位姿均成功读取；人工归零后的右臂运动预检通过；`motion_commands_sent=false` |
| 真机右臂单关节动作 | `d3_right_joint_step_benchmark.py --execute --confirm-right-only-joint-step --joint-index 1 --delta-deg 2.0 --duration-s 3.0` | 通过 | SDK 返回 `1`；J1 到位误差 `-0.0453°`；左臂最大漂移 `1.53e-05 rad`；运动后预检通过 |

## 新增可复现入口

```bash
cd /home/ubuntu/wt
bash scripts/evaluate/run_baseline_benchmarks.sh
```

该命令只运行软件/仿真基准和真机只读脚本的 dry run；不会连接真机。

真机只读基准必须由现场人员确认安全门后显式执行。它只读取版本、双臂配置/状态、关节位置/速度/力矩和法兰位姿；不调用复位、伺服、控制模式、夹爪、`move_*`、`track_*`、停止或急停接口：

```bash
cd /home/ubuntu/wt
export D3_CONTROLLER_ADDRESS='<控制器地址>'
.venv-sim/bin/python scripts/evaluate/d3_hardware_readonly_benchmark.py --execute
```

结果会写入 Git 忽略的 `artifacts/evaluation/d3-hardware-readonly.json`，其中不应包含真实 IP、凭据或其他敏感信息。

## 已完成的右臂软件准备

- 新增 `src.robots.d3.D3RightArmSafetyAdapter`：只暴露右臂关节点到点与受控跟随接口；
  不提供左臂、夹爪、使能、复位或控制模式切换。
- 默认点到点命令限制为单关节最大 `2°`、持续至少 `3 s`，并在每次发送前读取 SDK
  限位、关节反馈和右臂 POSITION/伺服状态。
- 连续 `track_right_joint()` 默认关闭；需独立验证全局 `task_stop()` 后才可显式启用。
- 增加 JSONL 命令审计入口、右臂配置模板、现场流程与安全清单条目。

上述适配器只进行了离线 fake-SDK 单元测试；硬件只读与单关节脚本的 dry-run 已确认
不会连接控制器或发出运动命令。它们不替代现场停止链路、夹爪或视觉标定。

## 现场待完成基准与通过门槛

| 项目 | 执行条件 | 通过门槛 |
| --- | --- | --- |
| 右臂其余单关节低速动作 | 每次均通过只读预检；实体急停可触达，最低速度，空载，现场监护 | 每次仅一个关节 `2°–3°`，方向、零位、软限位一致，左臂无命令 |
| 软件停止与超时 | 不带负载、恢复流程已验证 | 命令超时、断连和进程退出均停止或安全保持 |
| 夹爪与工具标定 | 原厂夹爪完成安装与量程标定 | 小幅开合无碰撞，工具坐标、量程和力限制已记录 |
| 相机与手眼标定 | 相机支架和线束完成检查 | 双腕相机按序列号稳定识别，内参与手眼误差满足任务阈值 |
| 连续跟随/VLA | 前述项和离线动作范围检查全部通过 | 有频率、速度、加速度、限位和陈旧命令超时保护；从单次短 episode 开始 |

详细现场操作依据 [实机安全检查清单](../operations/hardware-safety-checklist.md) 与 [开箱和首次连通指南](../operations/d3-hardware-bringup-and-integration.md)。
