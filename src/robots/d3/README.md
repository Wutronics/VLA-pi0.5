# D3 integration

`right_arm_safety.py` 是项目第一阶段的真机命令边界。它围绕供应商的
`carm.CArmDualBot` 建立一个**只允许右臂关节命令**的窄接口：

- `read_snapshot()`：只读状态、配置和限位；
- `arm_motion()`：重新读取状态后，由调用者显式授权；
- `move_joint(target_rad, duration_s=...)`：受限的低频点到点关节运动；
- `track_joint(target_rad)`：默认关闭；只有验证全局 `task_stop()` 后才可启用；
- `close()`：只断开。若已启用跟随且停止路径已验证，先调用 `task_stop()`。

它不提供左臂、夹爪、复位、伺服使能、`set_ready()` 或控制模式切换接口。现场人员
必须先通过上位机确认右臂使能、POSITION 模式、低速度、清场和实体急停可触达；软件
适配器不会替代这些步骤。

最小离线用法（不产生网络连接）：

```python
from src.robots.d3 import D3RightArmSafetyAdapter, RightArmSafetyConfig

# client 是已在现场明确创建的 CArmDualBot；构造 adapter 本身不发送命令。
adapter = D3RightArmSafetyAdapter(client, RightArmSafetyConfig())
snapshot = adapter.arm_motion()             # 只读预检
adapter.move_joint(snapshot.right_joint_position_rad, duration_s=3.0)
adapter.close()
```

完整的现场顺序、配置项和尚未满足的门禁见
[右臂安全适配器操作说明](../../../docs/operations/d3-right-arm-safety-adapter.md)。
