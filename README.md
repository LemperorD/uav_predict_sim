# 无人机轨迹预测数值仿真

生成运动轨迹，通过引用的运动学仿真器输出里程计，加入观测噪声后执行 IMM 轨迹预测，并通过 Pangolin 实时显示轨迹与误差曲线。

- [编译、运行与结果说明](src/predictor/README.md)
- [数值仿真配置](src/predictor/config/simulation.yaml)
- [Pangolin 显示配置](src/predictor/config/visualization.yaml)

在工作区完成编译后执行 `scripts/run_simulation.sh`。每轮运行前，在数值配置中设置新的 `output_dir`。
