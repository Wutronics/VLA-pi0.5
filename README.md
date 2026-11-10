# CVTE D3 × π0.5 Pick and Place

本项目面向 CVTE D3 右臂药盒抓取与放置实验，覆盖外骨骼遥操作、LeRobot 数据采集、π0.5 微调和受控真机推理。

## 当前状态

仓库已完成方案复核和开发骨架初始化，尚未完成实机联调。当前推荐基线为：

1. 使用 CVTE 的 `carm` SDK 与 `carm-lerobot` 中的 D3 驱动采集数据；
2. 第一阶段只允许右臂执行动作，左臂保持禁用；
3. 使用 LeRobot 格式保存数据；
4. 优先验证 LeRobot 的 π0.5 端到端路径，保留切换到 OpenPI 训练/策略服务器的接口。

CVTE D3 驱动仍有上机前必须修复或验证的安全项，详见[计划评审](docs/reviews/2026-11-10-plan-review.md)。

## 无真机时的 D3 仿真

项目已接入 CVTE 官方 `carm-mujoco` D3 模型，并提供仅暴露右臂动作的练习入口。首次运行：

```bash
bash scripts/simulation/setup_simulation.sh
bash scripts/simulation/run_d3_sim.sh --demo all --render
```

服务器无图形界面时，可运行同一套无渲染测试：

```bash
bash scripts/simulation/run_d3_sim.sh --demo all
.venv-sim/bin/python -m unittest tests.unit.test_d3_right_arm_sim -v
```

具体练习步骤、预期结果和仿真边界见 [D3 MuJoCo 仿真指南](docs/setup/d3-mujoco-simulation.md)。仿真通过不代表已经满足真机安全门。

## 服务机快速开始

```bash
git clone https://github.com/Wutronics/VLA-pi0.5.git
cd VLA-pi0.5
bash scripts/bootstrap_dependencies.sh
```

脚本会把锁定版本的外部依赖下载到被 Git 忽略的 `third_party/`。若需要同时下载 OpenPI：

```bash
bash scripts/bootstrap_dependencies.sh --with-openpi
```

依赖下载完成后，请先阅读环境说明和安全检查清单；当前仓库不提供“一键上机运动”脚本。

## 目录

```text
configs/                  无密钥的机器人、遥操作和训练配置模板
data/                     原始、处理后和小型样例数据的存储位置（数据默认不提交）
docs/                     方案、评审、环境、数据规范和操作文档
artifacts/                训练日志、检查点和评测结果（默认不提交）
scripts/                  依赖安装、自检、采集、转换、训练和评测入口
src/                      项目自有适配、映射、安全和策略集成代码
tests/                    单元测试、数据契约测试和硬件测试
third_party/              锁定的外部仓库工作副本（默认不提交）
```

## 关键文档

- [实验实施方案](docs/teleoperation/exoskeleton-teleoperation-plan.md)
- [现有计划评审](docs/reviews/2026-11-10-plan-review.md)
- [服务机环境与依赖](docs/setup/server-environment.md)
- [D3 常用 API 与控制方式](docs/operations/d3-common-api-and-control.md)
- [数据集约定](docs/data/dataset-spec.md)
- [实机安全检查清单](docs/operations/hardware-safety-checklist.md)
- [外部资料与锁定版本](docs/references.md)

## 数据与凭据规则

- 不提交密码、令牌、私钥、设备认证信息、真实 IP 或 `.env`。
- 不把原始 episode、视频、深度图、模型权重或训练日志直接提交到 Git。
- 数据集应放在 `data/` 对应目录或外部存储中；模型与日志放在 `artifacts/`。
- 仅提交脱敏的小型样例、数据清单和可复现配置。
