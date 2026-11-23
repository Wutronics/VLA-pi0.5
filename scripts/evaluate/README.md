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

## 右臂单关节基准

现场已完成右臂 J1 正向 `2°`、`3 s` 的低速基准。脚本会以实时反馈作为起点，拒绝越限、非位置模式、未使能或预检失败状态；它不调用复位、伺服、控制模式、左臂或夹爪接口。

```bash
export D3_CONTROLLER_ADDRESS='<控制器地址>'
.venv-sim/bin/python scripts/evaluate/d3_right_joint_step_benchmark.py \
  --execute --confirm-right-only-joint-step \
  --joint-index 1 --delta-deg 2.0 --duration-s 3.0
```

每次真机动作前都应重新执行只读基准；结果写入 Git 忽略的 `artifacts/evaluation/`。
