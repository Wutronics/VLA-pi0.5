# 服务机环境与依赖

## 建议环境拆分

不要把采集、训练和 OpenPI 强行安装到同一个 Python 环境。

| 环境 | 用途 | 基线要求 |
| --- | --- | --- |
| `d3-collect` | D3、相机、外骨骼和 LeRobot 数据采集 | CVTE `carm-lerobot` 锁定版本要求 Python >= 3.12；另装 `carm` |
| `pi05-train` | LeRobot π0.5 微调与离线推理 | 复用锁定的 `carm-lerobot`，按其 `pi` extra 安装 |
| `openpi`（可选） | OpenPI 微调/策略服务器对照路径 | 锁定版本要求 Python >= 3.11，JAX CUDA 12 与指定 PyTorch/Transformers 版本 |

CVTE SDK Demo 的传统推荐组合是 Ubuntu 20.04、Python 3.8、ROS Noetic/Foxy，而当前 `carm-lerobot` 要求 Python 3.12。`carm` 声明支持 Python 3.6+，但必须在服务机上实际验证 Python 3.12、控制器固件和 WebSocket 通信兼容性。这是环境冻结前的阻断项。

## 首次 clone

```bash
git clone https://github.com/Wutronics/VLA-pi0.5.git
cd VLA-pi0.5
bash scripts/bootstrap_dependencies.sh
```

可选下载 OpenPI：

```bash
bash scripts/bootstrap_dependencies.sh --with-openpi
```

脚本会检出不可变 commit，并验证实际 SHA。第三方源码位于 `third_party/`，不应提交回本仓库。

## 采集环境建议步骤

```bash
conda create -n d3-collect python=3.12 -y
conda activate d3-collect
conda install ffmpeg -c conda-forge
python -m pip install --upgrade pip
python -m pip install carm
python -m pip install -e 'third_party/carm-lerobot[pi]'
```

上面只是软件安装，不代表允许连接并驱动机械臂。安装后依次完成：

1. 记录 GPU、CUDA、驱动、Python、`carm`、控制器固件和相机驱动版本；
2. 运行只读 SDK 连接和状态查询；
3. 修复/包装 CVTE D3 驱动的自动复位与连接状态问题；
4. 完成 `docs/operations/hardware-safety-checklist.md`；
5. 才能在隔离工位进行低速单关节测试。

## 本地配置

真实控制器地址和密钥不要写入提交文件：

```bash
export D3_CONTROLLER_ADDRESS='ws://<controller-host>:<port>'
cp configs/robot/d3-right-arm.example.yaml configs/robot/d3-right-arm.local.yaml
```

`*.local.yaml` 已被忽略。不要把 shell history、`.env` 或机器密码复制到仓库。
