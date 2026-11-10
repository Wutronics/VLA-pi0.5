# Scripts

- `bootstrap_dependencies.sh`：按 `dependencies.lock.toml` 下载并校验外部依赖。
- `self_check/`：后续放置只读连通性、SDK 版本、相机和时钟检查。
- `collect/`：后续放置经安全评审的数据采集入口。
- `convert/`：后续放置数据清洗与 LeRobot 转换工具。
- `train/`：后续放置 π0.5 训练入口和配置校验。
- `evaluate/`：后续放置离线评测和受控真机评测入口。

任何可能产生机械臂动作的脚本都必须显式要求 `--enable-motion`，默认只读或 dry-run。
