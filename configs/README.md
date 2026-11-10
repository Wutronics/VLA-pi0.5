# Configuration

这里只提交无密钥模板。真实设备地址、令牌、操作者信息和本地路径应通过环境变量或 `*.local.yaml` 提供；本地配置已被 `.gitignore` 排除。

当前 YAML 是项目级配置契约，不保证能被 CVTE 或 LeRobot CLI 直接读取。实现适配器后再增加严格的 schema 校验。
