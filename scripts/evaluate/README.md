# Evaluation scripts

## 软件/仿真基准

```bash
bash scripts/evaluate/run_baseline_benchmarks.sh
```

该入口运行 D3 右臂 MuJoCo 演示、单元测试和硬件只读脚本的 dry run；默认不会连接或控制真机。

## 真机只读基准

仅在现场安全检查完成后，由现场人员显式执行：

```bash
export D3_CONTROLLER_ADDRESS='<控制器地址>'
.venv-sim/bin/python scripts/evaluate/d3_hardware_readonly_benchmark.py --execute
```

该脚本只读版本、状态、配置、关节反馈和法兰位姿，不调用运动、夹爪、伺服、控制模式或停止接口。更高风险的单关节、夹爪、相机、外骨骼和 VLA 基准应按实机安全检查清单逐项完成。
