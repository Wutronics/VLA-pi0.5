# 外部资料与版本基线

核对日期：2026-11-10。

## 必需资料

| 资料 | 用途 | 本项目基线 |
| --- | --- | --- |
| [CVTE CArm SDK Demo](https://github.com/cvte-robotics/carm_demo) | `carm` SDK、Python/C++/ROS 示例和 API 说明 | `5e6e4a5bee16222194392c76a12836ba15cf5136` |
| [CVTE carm-lerobot](https://github.com/cvte-robotics/carm-lerobot) | D3 LeRobot 驱动、数据采集和 π0.5 路径 | `31b1349bc1906765dd522324659a07bd7d281c10` |
| [CVTE carm_grasp](https://github.com/cvte-robotics/carm_grasp) | 眼在手相机、手眼/夹爪标定与抓取示例 | `870ee61747b20d70732673cf4ae932a6c5e822e1` |
| [CArm 用户文档中心](https://cvte-bot.feishu.cn/wiki/GyGbwKeWMiqEfDk80QXc9zeCnjf) | 产品、控制器、SDK 下载和最新说明 | 每次上机前人工核对 |
| [CVTE 柔性机械臂产品页](https://www.cvte.com/product/flexiblemanipulator) | 产品信息 | 参考，不作为 API 契约 |

## π0.5 上游

| 资料 | 用途 | 本项目基线 |
| --- | --- | --- |
| [Physical Intelligence OpenPI](https://github.com/Physical-Intelligence/openpi) | π0.5 上游实现、微调和策略服务器 | `215abfb217dbac7d5f1273282331b9b1866c0479`（可选） |
| [Hugging Face LeRobot](https://github.com/huggingface/lerobot) | LeRobot 数据和 π0.5 实现参考 | `81b8c5bfdd5591ec59e512cfc7ac6db8151c70ee`（参考） |

精确下载版本记录在 `dependencies.lock.toml`。不要直接追踪外部仓库的 `main` 或 `release` 分支进行实验；升级依赖时应新建数据/模型兼容性验证记录。

## 已确认接口事实

- D3 是每臂 7 关节的双臂设备，CVTE LeRobot 驱动使用 `carm.CArmDualBot`。
- CVTE 驱动提供 `joint`、`pose`、`both` 三种记录模式；`both` 在其当前实现中不支持推理。
- `carm` Python 包通过 WebSocket 与控制器通信，SDK Demo 提供状态、关节/末端、夹爪、急停和任务停止接口。
- LeRobot π0.5 支持相对动作预处理；是否启用必须与数据字段、归一化统计和真机后处理一起验证。

上述事实来自锁定版本的源码，仍需在实际交付的 D3、控制器固件和 SDK 版本上复核。
