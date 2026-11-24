# 外骨骼人形机器人数据采集 (record_lerobot)

## CVTE D3 右臂适配

本仓库新增 `robot_impl/d3_right_robot.py` 和 `config/config_d3_right.yaml`。D3 配置默认
为 dry-run：只检查外骨骼与 7 维数据链路，不连接或移动机器人。

```bash
cd /home/ubuntu/wt
python teleoperator/main.py \
  --config teleoperator/config/config_d3_right.yaml
```

D3 模式使用参考姿态上的相对增量映射，白键不会执行旧机器人的归位动作。真机启用前
必须完成逐关节映射和 `task_stop()` 验证，详见
[`docs/teleoperation/d3-exoskeleton-adaptation.md`](../docs/teleoperation/d3-exoskeleton-adaptation.md)。

通过外骨骼设备和人形机器人设备，采集关节数据 + 摄像头图像，生成标准 LeRobot v3.0 格式数据集。

> 基于 `lerobot>=0.6.0`，数据集格式为 codebase_version v3.0。

---

## 1. 前置工作

### 1.1 Python 环境

本项目使用仓库锁定的 Python 3.12 `carm-lerobot`，不要安装到 Python 3.10 的
`.venv-sim`。创建 `d3-collect` 环境后安装：

```bash
conda create -n d3-collect python=3.12 -y
conda activate d3-collect
python -m pip install -e third_party/carm-lerobot
python -m pip install -r teleoperator/requirements-d3-collect.txt
python -m pip install carm
```

> `qnbot_sdk_v1.2/` 是本地交付资产，不提交到 Git；请按下节放入 `teleoperator/`。

Linux 下配置串口权限后需重新登录，或在当前终端执行 `newgrp dialout`：

```bash
sudo usermod -aG dialout "$USER"
newgrp dialout
```

D3 配置默认使用 `port: auto`，优先匹配 `/dev/serial/by-id/*Qnbot*`，因此设备重插后
即使从 `ttyACM0` 变为 `ttyACM1` 也不需要改配置。

### 1.2 外骨骼 SDK

将 `qnbot_sdk_v1.2/` 目录放在项目根目录下，确保以下文件存在：

```
remapping_data_collect_v3/
├── qnbot_sdk_v1.2/
│   ├── qnbot_sdk.py        # 外骨骼通信核心
│   ├── qnbot_cli.py
│   └── cli_utils.py
├── read_exo.py              # 外骨骼数据读取器
├── read_robot.py            # 机器人状态读取器
├── record_lerobot.py        # 主采集脚本 ← 本文档介绍的脚本
└── README.md
```

### 1.3 机器人 SDK（Fourier Aurora）

在机器人NUC上安装 `fourier_aurora_client`。如果暂时没有，`read_robot.py` 会自动使用全零占位数据，不影响采集流程测试。

### 1.4 启动 Aurora 服务

在机器人 NUC 上，通过 Docker 启动 Aurora 核心服务：

```bash
# 1. 检查容器是否存在
docker ps -a | grep fourier_aurora

# 2. 如果容器未运行，启动它
docker start fourier_aurora_server

# 3. 进入容器
docker exec -it fourier_aurora_server bash

# 4. 在容器内启动 AuroraCore
AuroraCore --config config/config.yaml
```

确认看到 `Start robot control [aurora_main]` 输出后，Aurora 服务即正常运行。

### 1.5 串口权限

**Linux：**
```bash
# 将用户加入 dialout 组（重启生效）
sudo usermod -a -G dialout $USER

# 或临时给串口权限
sudo chmod 666 /dev/ttyACM0
```


---

## 2. 执行步骤

### 2.1 启动采集

```bash
cd remapping_data_collect_v3
python record_lerobot.py
```

- 有摄像头：自动启用流式编码（`streaming_encoding=True`），边录边编码为 mp4
- 无摄像头：自动跳过视频，仅采集关节/摇杆/按钮数据

> 流式编码下 `save_episode()` 只需几十毫秒，无需手动切换模式。

### 2.2 录制操作

| 操作 | 方式 |
|------|------|
| 开始录制 | 按下外骨骼**左手红色按钮**（toggle） |
| 停止录制 | 按一次红色按钮 → 停止当前 episode，保存+编码完成 |
| 录制下一个 | 再按一次红色按钮 → 开始新 episode |
| 退出程序 | `Ctrl + C` |

### 2.3 运行中输出说明

```
[▶] episode 0 开始录制                    ← 开始录制
[耗时] add_frame: avg=1.2ms ... (n=150)  ← 每 5 秒打印 add_frame 耗时
[■] episode 0 录制停止 (1800 帧,          ← 停止录制，保存耗时包含视频编码
     采集 fps=29.8, 保存耗时=85ms)
[退出] 正在 finalize 数据集...            ← Ctrl+C 后 flush 所有写入器
[退出] finalize 完成
```

---

## 3. 输出文件

### 3.1 目录结构

采集脚本会在项目根目录下生成以时间戳命名的数据目录：

```
remapping_data_collect_v3/
└── datasets_v3_20260721_151503/    ← 一次采集会话
    ├── data/
    │   └── chunk-000/
    │       ├── file-000.parquet    ← 关节数据、时间戳、任务等
    │       └── ...
    ├── videos/
    │   └── observation.images.camera_0/
    │       └── chunk-000/
    │           ├── file-000.mp4    ← 摄像头视频（AV1 编码）
    │           └── ...
    └── meta/
        ├── info.json              ← 数据集元信息（codebase_version: v3.0）
        ├── stats.json             ← 全局数据统计
        ├── tasks.parquet          ← 任务标签（Parquet）
        └── episodes/
            └── chunk-000/
                └── file-000.parquet ← episode 统计（Parquet）
```

> v3.0：`episodes.jsonl` / `tasks.jsonl` 改为 Parquet 格式；不再有 `images/` 临时目录。

### 3.2 每个 episode 保存哪些数据

| 字段 | 维度 | 说明 |
|------|------|------|
| `observation.state` | 10 | 机器人当前关节角度（左右臂各 5 关节） |
| `action` | 10 | 外骨骼目标关节角度 |
| `observation.joystick` | 6 | 左右摇杆值 |
| `observation.button` | 8 | 左右按钮状态 |
| `observation.images.camera_0` | 640×480×3 | 摄像头 RGB 视频帧 |

### 3.3 数据集大小参考

| 项目 | 参考值 |
|------|--------|
| 单帧元数据（关节+摇杆+按钮） | ~300 字节 |
| 单帧视频（AV1 CRF=30, 640×480） | ~8-15 KB |
| 1 分钟 episode（1800 帧） | Parquet ~1 MB + 视频 ~15-25 MB |

---

## 4. 文件清单

| 文件 | 说明 |
|------|------|
| `record_lerobot.py` | 主采集脚本（lerobot 0.6.0 / v3.0 格式） |
| `read_exo.py` | 外骨骼数据读取（串口通信） |
| `read_robot.py` | 机器人状态读取（Aurora SDK） |
| `qnbot_sdk_v1.2/` | 外骨骼通信 SDK |
