# 三维无人机 IMM-KF 预测与仿真

## 构建与运行

依赖 Ubuntu 22.04 / ROS2 Humble、Eigen3、Pangolin、Python3 开发库与系统 Matplotlib、FFmpeg。Pangolin 需提前安装，CMake 可使用 `-DPangolin_DIR=/path/to/lib/cmake/Pangolin` 指定。matplotlib-cpp 已随 `src/visualize/third_party` 提供，并保留 MIT 许可证。

在工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
# 已构建的 EGO 工作区也需要 source install/setup.bash
colcon build --packages-up-to predictor prediction_visualize poscmd_2_odom \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3
scripts/run_simulation.sh
```

默认运行 30 秒；2 秒预热后开始发送轨迹。界面左侧为三维真实轨迹（绿）、带噪观测（灰）、滤波估计（蓝）、未来轨迹（橙），右侧显示位置/预测误差、协方差对角线、模型概率。鼠标可旋转、平移、缩放三维视角。正常运行结束后自动退出界面、导出曲线和 MP4。提前关窗会使整个流程提前停止并报告失败。

默认输出为工作区 `outputs/demo/`。重复运行前，在配置里改用新的 `output_dir`；运行器拒绝覆盖已有数据。

```bash
scripts/run_simulation.sh --config /absolute/path/to/my_simulation.yaml
scripts/run_simulation.sh --config /absolute/path/to/my_simulation.yaml --headless
# 用已有 CSV/帧重新生成图表、统计与视频；无图形运行的数据应同时传 --headless
scripts/run_simulation.sh --config /absolute/path/to/my_simulation.yaml --finalize-only
```

`--headless` 完整数值仿真与 matplotlib-cpp 绘图，不生成 Pangolin 窗口或视频。仅需 ROS 启动时可运行 `ros2 launch predictor simulation.launch.py config:=/absolute/path/to/config.yaml`，但结束后的统计/绘图/视频编码由上述脚本负责。

## 输入与配置

参数示例位于 `config/simulation.yaml`。`output_dir`、`input_csv` 和 `ffmpeg_path` 的相对路径以 YAML 所在目录为基准，其余输出路径以 `output_dir` 为基准，均可设为绝对路径。CSV、配置快照、ROS 日志、Matplotlib 缓存、图像、视频和录像帧目录的名字/位置都可配置。路径可含空格。每次运行保存解析后的 YAML 以便复查。

默认使用 EGO Planner Swarm 自带的 `poscmd_2_odom` 运动学无人机仿真，链路为：

```
kinematic_source → PositionCommand → EGO fake_drone → Odometry
                                                    ↓
                              加入观测噪声 → IMM → estimate / prediction / metrics
```

输入只有位置、速度、加速度与偏航等运动量，没有力/力矩或推力输入。此演示使用 EGO 的运动学仿真器，不启动完整 EGO 障碍地图、路径规划器或动力学控制器。`odometry_topic`、`command_topic` 可配置；接入已有仿真/实测里程计时可单独运行 `predictor_node --ros-args --params-file <yaml> -r odometry:=<topic>`，没有 truth 时不应把输出误差视作真实精度。

- `trajectory`：`figure8` 三维八字、`straight` 匀速、`acceleration` 恒加速度。
- `input_csv`：非空时替代内置轨迹，首行为 `t,px,py,pz,vx,vy,vz,ax,ay,az`。世界系、SI 单位、时间从 0 开始严格递增，覆盖整个 `duration`；采样间逐列线性插值。使用运动学一致的数据。
- `sample_rate`：运动指令频率；EGO 原有里程计输出频率为 100 Hz。
- `inject_noise`：对位置和世界系速度分别加入独立零均值高斯噪声；`position_noise_std`（m）、`velocity_noise_std`（m/s）、`noise_seed` 可调。种子固定随机序列，ROS 实时调度仍有微小时间差。
- `cv_accel_density`、`ca_jerk_density`：CV 的白加速度与 CA 的白 jerk 过程噪声谱密度；`stay_probability`：对称模型转移矩阵对角元素。
- `prediction_step × prediction_steps`：预测总时域，默认 1 秒。预测只使用当前及过去的带噪运动观测。
- 噪声标准差需为正；关闭噪声使用 `inject_noise: false`，此时滤波器仍使用配置的正测量协方差。
- `record_video`、`video_fps`、`window_width/height`：窗口录像；宽高须为正偶数。帧以 PNG 保存，结束后 FFmpeg 编码 H.264。帧保留供复查，长时间录像请预留磁盘空间。

## 算法和误差含义

统一状态为 `[px,py,pz,vx,vy,vz,ax,ay,az]`。CV 分支把加速度重置为零，CA 分支保持加速度；每次更新执行 Markov 概率传播、条件状态/协方差混合、KF 预测/观测更新和对数似然模型概率更新。协方差用 Joseph 形式更新；融合协方差包含模型均值差异。未来推演复制当前 IMM，在无未来观测条件下继续交互混合和传播，不修改在线滤波器。

`estimate` 为带位置/速度协方差的 Odometry；`prediction` 为未来带时间戳的 Path。`metrics` 为 19 维数组，列顺序与 samples CSV 一致。EGO 仿真器把加速度写入 twist.angular 的旧约定没有被预测器使用；预测器只读取位置与线速度。

- `samples.csv`：时间、真实位置、含噪位置、估计位置、位置标准差、CV/CA 概率、位置误差、最近到期的预测误差、速度误差、位置 NEES。
- `predictions.csv`：预测发出时间、目标时间、预测位置、按目标时间插值对齐的真实位置、预测误差与目标位置标准差。每次回调记录一个完整预测时域终点；末尾尚未等到真实值的预测不计入误差。收到仿真结束通知后停止记录，避免把仿真器保持的最后位置当成后续真值。
- `summary.png`：matplotlib-cpp 从 CSV 生成的轨迹、误差、协方差和概率曲线。
- `metrics.json`：观测与滤波位置 RMSE、速度 RMSE、预测时域终点 RMSE、平均 NEES 和每轴 2σ 覆盖率。滤波聚合指标排除前 2 秒预热，完整原始记录保留。

无穷/NaN 输入及重复、逆序时间戳被跳过；超过 `max_time_gap` 的正向时间间隔会重置滤波器。仿真真值用于验证和绘图，不反馈给滤波状态；预测器使用的测量是加入噪声后的位置和速度。

## 验证

```bash
ctest --test-dir build/predictor --output-on-failure
# 在 source 工作区后，独立 DDS 域可避免与已有 ROS 任务串话
ROS_DOMAIN_ID=65 scripts/acceptance_check.py
```

核心测试覆盖长序列有限性、概率归一化、协方差对称/半正定、噪声抑制、CV 外推、CA 模型响应、参数拒绝和预测不修改在线状态。集成检查真实启动 EGO fake_drone，覆盖带噪匀速、恒加速度、无噪三维轨迹、含空格 CSV 输入和自定义嵌套输出路径。

当前噪声参数是演示起点。`metrics.json` 同时输出 NEES 与覆盖率用于协方差校准；默认参数不保证每个运动场景达到标称 2σ 覆盖率。录像记录每帧的实际采集时间，并按可变帧率编码；渲染低于目标帧率时仍保留真实播放时长。`frame_timestamps_file` 和 `video_manifest_file` 可配置时间表与编码清单位置。
