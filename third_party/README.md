# Third-party dependencies

外部源码由 `scripts/bootstrap_dependencies.sh` 按 `dependencies.lock.toml` 的提交号下载，本目录不提交依赖内容。

当前必需依赖包括 D3/LeRobot 驱动、SDK 示例、抓取参考，以及 CVTE 官方 D3 MuJoCo 模型。请勿直接修改这里的源码；项目适配代码放在 `src/`。

运行 `bash scripts/bootstrap_dependencies.sh` 后，锁定版本的外部仓库会下载到本目录。下载内容被 `.gitignore` 排除，不会重复提交供应商或上游代码。

版本来源见仓库根目录的 `dependencies.lock.toml`。
