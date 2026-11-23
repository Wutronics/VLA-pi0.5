# 外骨骼到 D3 右臂适配说明

## 结论

原程序面向 XOIP/GR3 双臂机器人，不能直接连接 D3。适配后的第一阶段只使用外骨骼
右臂 7 个关节，只记录/控制 D3 右臂 7 个关节；左臂数据不会进入 D3 命令路径。
默认配置为 `dry_run: true`，可先验证外骨骼串口、按钮状态机、7 维数据 schema 和
LeRobot 录制，而不连接机器人。

## 原程序的关键风险

| 原行为 | 对 D3 的风险 | 适配处理 |
| --- | --- | --- |
| 14 维双臂绝对关节目标 | 会把另一种本体的零位和坐标定义直接套用到 D3 | 仅取右臂 7 维，并采用参考姿态上的相对增量 |
| 白键自动归位 | 归位数值来自旧机器人 | D3 模式下白键不产生运动 |
| 红键先绝对对齐再持续控制 | 首次动作可能幅度很大 | 红键只捕获外骨骼/D3参考姿态，不产生动作 |
| 循环持续发送且忽略异常 | 通信中断后可能保留旧目标 | 250 ms 看门狗触发经验证的 `task_stop()` |
| action 始终记录 14 维原始外骨骼值 | 与 D3 observation 维度和真实命令不一致 | action 记录映射后的 D3 右臂 7 维目标 |
| 相机与关节采集异步 | 帧间存在时延 | 当前保留结构，正式采集前需测量并记录时间偏差 |

## 映射语义

进入遥操作时记录外骨骼参考角 `q_exo_ref` 和 D3 当前反馈 `q_d3_ref`，之后计算：

```text
q_d3_target = q_d3_ref + (q_exo - q_exo_ref) × sign × gain
```

`exo_indices` 处理关节排列，`sign` 处理正方向，`gain` 缩小或放大动作。默认 gain 为
`0.5`，但 sign、零位和关节对应关系仍是未验证值；完成逐关节标定前，
`mapping_verified` 必须保持 false。

D3 配置关闭 `ExoReader` 的旧机器人绝对角度裁剪，最终映射目标由 D3 控制器按 7 轴
限位和每周期最大 `1°` 变化检查；真机模式还会再次使用 SDK 实际限位检查。

## 离线运行

先安装数据链路依赖，并确保供应商 `qnbot_sdk_v1.2/` 位于 `teleoperator/`：

```bash
cd /home/ubuntu/wt
conda activate d3-collect
python teleoperator/main.py \
  --config teleoperator/config/config_d3_right.yaml
```

默认行为：读取外骨骼，D3 状态使用 7 维零值占位，白键不归位，红键只建立参考映射，
蓝键开始/停止录制。不会创建 `carm` 客户端。

## 真机启用门

只有完成以下现场测试后，才能把配置中的三项门禁改为 true：

1. 逐关节确认外骨骼右臂与 D3 的排列、正方向和 gain；
2. 独立验证 `task_stop()` 能终止跟随，并确认恢复流程；
3. 验证 10 Hz 初始命令率、`1°` 单帧变化限制和 250 ms 看门狗；
4. 空载、低速、实体急停可触达且有现场监护。

真机运行还要求两个环境变量：

```bash
export D3_CONTROLLER_ADDRESS='<controller-address>'
export D3_TELEOP_EXECUTE=I_UNDERSTAND_D3_RIGHT_ARM_ONLY
```

这些门禁只用于避免误启动，不替代现场风险评估。首次真机映射验证应逐关节进行，不能
直接做七关节自由遥操作。

## 数据契约

| 字段 | 维度 | 含义 |
| --- | ---: | --- |
| `observation.state` | 7 | D3 右臂实际关节角，rad |
| `action` | 7 | 经相对映射后实际请求的 D3 右臂目标，rad |
| `observation.joystick` | 6 | 外骨骼双侧摇杆原始字段 |
| `observation.button` | 8 | 外骨骼双侧按钮，按下为 1 |

正式数据集不能混入 dry-run 的零状态帧；dry-run 只用于检查软件链路。
