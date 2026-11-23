# D3 右臂安全适配器与现场流程

适用范围：本项目第一阶段的 D3 右臂关节测试、遥操作前置验证和数据采集准备。

`src.robots.d3.D3RightArmSafetyAdapter` 是对交付版 `carm.CArmDualBot` 的窄包装。
它只读左臂诊断反馈，但**没有任何左臂动作接口**；不包含夹爪、复位、使能、
控制模式切换或 `set_ready()`。因此它不是通用 SDK 的替代品，也不代替实体急停和
上位机的安全状态确认。

## 1. 已实现的门禁

| 门禁 | 当前行为 |
| --- | --- |
| 连接/构造 | 适配器构造无网络与运动副作用；`from_carm()` 才显式创建 SDK 客户端 |
| 控制范围 | 仅调用 `move_right_joint()` 或经明确启用后的 `track_right_joint()` |
| 授权 | 进程内必须先调用 `arm_motion()`；该调用重新读取状态与 SDK 关节限位 |
| 状态 | 默认要求右臂 `servo=1`、`fsm_state=POSITION`、`motion_state=1` |
| 点到点 | 默认每关节最大变化 `2°`，时长至少 `3 s`，目标和反馈必须在 SDK 限位内 |
| 连续跟随 | 默认关闭；还要求 `global_task_stop_verified=true` |
| 日志 | 可选 JSONL，记录授权、请求目标、完成反馈、拒绝和断开 |

最大单步是软件层的保守起点，不是硬件速度/加速度限制的替代品。所有限位与状态仍以
控制器的交付配置和现场读数为准。

## 2. 现场使用顺序

1. 固定底座、清空工作空间；现场人员可立即按实体急停。
2. 在上位机确认：仅右臂使能、POSITION 模式、低速度；左臂不参与动作。
3. 先运行只读检查，保存本次结果：

   ```bash
   export D3_CONTROLLER_ADDRESS='<controller-address>'
   .venv-sim/bin/python scripts/evaluate/d3_hardware_readonly_benchmark.py --execute
   ```

4. 将 `configs/robot/d3-right-arm.example.yaml` 复制为本地、Git 忽略的运行配置，记录
   本次速度、关节限位来源和日志位置。
5. 仅在现场观察下，为单个小步点到点测试创建适配器、调用 `arm_motion()`，再调用
   `move_joint()`。每次命令都从刚读到的 7 维 `right_joint_position_rad` 生成目标。
6. 完成后调用 `close()`；记录结果和任何异常。未验证停止路径前，不要启用连续跟随。

现有经实机验证的单关节脚本仍可用于低速、单次 J1 步进基准；其执行必须显式确认：

```bash
.venv-sim/bin/python scripts/evaluate/d3_right_joint_step_benchmark.py \
  --execute --confirm-right-only-joint-step --joint-index 1 --delta-deg 2 --duration-s 3
```

不要把该命令改为循环，也不要在无人监护时运行。

## 3. 控制语义与阶段边界

- `move_joint()`：单个终点的点到点动作。用于低速准备位和逐关节确认。
- `track_joint()`：高频目标流。遥操作和策略推理可能需要它，但它会放大失联、过期
  指令和尺度错误的风险；在独立验证 `task_stop()`、控制周期、速度/加速度限制和
  看门狗前保持关闭。
- 夹爪和位姿控制未接入该适配器。安装/标定夹爪、工具 TCP 与相机手眼后，再增加各自
  独立的受限接口与测试。

## 4. 仍未完成的前置门

- 实体急停和软件 `task_stop()` 的独立、受控验证；
- 右臂程序化 J2–J7 单步覆盖（上位机手动检查不替代带日志的程序化证据）；
- 夹爪安装、零位/开口/力标定；
- 相机安装、时间同步与手眼标定；
- 按真实速度/加速度与控制频率建立项目级限幅；
- 完成上述事项后，才评估外骨骼 `track_*` 与 VLA 受控推理。

在任何一项未通过时，保持 `tracking_enabled: false`，并继续使用只读或单次受监护的
点到点测试。
