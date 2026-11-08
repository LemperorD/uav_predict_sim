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

- `trajectory`：七种解析轨迹，详细参数和公式见下表。默认三维八字与原版一致；旧 YAML 不增加新参数也可运行。
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


## 轨迹选择与调参

修改 `config/simulation.yaml`，设置 `trajectory` 并为每次运行选择新的 `output_dir`，然后执行 `scripts/run_simulation.sh --headless`（需要窗口时去掉该选项）。这些参数改变发送给仿真无人机的运动指令，里程计经过原有 IMM 链路完成预测。

所有位置/速度/加速度都在世界坐标系中，采用 m、s、rad。三元组按 x/y/z 排列，必须恰有三个有限数值。

| trajectory | 运动与适用场景 | 有效参数 |
| --- | --- | --- |
| `hover` | 固定位置，检验静止时噪声抑制 | `trajectory_origin` |
| `straight` | 匀速直线，检验 CV 外推 | origin、velocity |
| `acceleration` | 恒加速度，检验 CA 响应 | origin、velocity、acceleration |
| `circle` | 水平圆周，检验持续转弯 | origin（圆心）、radius、angular_speed、circle_phase |
| `helix` | 水平圆周加匀速升降 | 圆周参数、vertical_speed |
| `figure8` | 三维八字，检验变曲率运动 | origin（中心）、amplitude、frequency、phase；y 频率固定为 x 的两倍 |
| `sinusoid` | 三轴独立正弦，可配置波浪或 Lissajous 运动 | origin（中心）、amplitude、frequency、phase |

表中缩写均带 `trajectory_` 前缀。`origin` 默认 `[0,0,2]`，匀速的 `velocity` 默认 `[1,0.4,0]`，恒加速度的初速度默认 `[0,0.2,0]`，`acceleration` 默认 `[0.2,0,0]`。显式设置 `trajectory_velocity` 可覆盖初速度。

直线/加速运动满足 `p = origin + velocity*t + acceleration*t²/2`（直线取零加速度）。正弦运动每轴为 `origin + amplitude*sin(frequency*t + phase)`；八字使用 `frequency_y = 2*frequency_x`，忽略配置中的第二个频率。其余频率为独立角频率（rad/s），周期为 `2π/|frequency|`；允许零频率、负频率和零幅值。相位不同或退化幅值可能改变八字的外形。

圆周/螺旋为 `x = origin_x + radius*cos(angular_speed*t + circle_phase)`，`y = origin_y + radius*sin(...)`；圆周保持高度，螺旋为 `z = origin_z + vertical_speed*t`。默认半径 4 m、角速度 0.35 rad/s、相位 0、爬升速度 0.2 m/s。半径必须为正；负角速度反转方向，负爬升速度下降。圆心与起点不同，起点位于圆周上。所有轨迹同步发送解析位置、一阶速度和二阶加速度。

仿真器按所选轨迹在 t=0 的位置初始化（包含相位；CSV 使用首行位置），避免首条指令到达前发布旧起点。初始静止速度在收到首条运动指令后更新。

例如，保持其他参数，改成半径 3 m、每 20 秒一圈、以 0.1 m/s 爬升的螺旋：

```yaml
    output_dir: ../../../outputs/helix_trial_01
    trajectory: helix
    trajectory_origin: [0.0, 0.0, 2.0]
    trajectory_radius: 3.0
    trajectory_angular_speed: 0.3141592653589793
    trajectory_vertical_speed: 0.1
```

`input_csv` 非空时覆盖内置轨迹，允许忽略的轨迹名称；其格式与线性插值规则保持不变。内置轨迹参数在启动时校验，非法名称、错误向量长度、非有限数值或无效半径会返回非零状态。轨迹不负责避障或高度约束，请按仿真场景选择参数。

## C++ 格式

根目录 `.clang-format` 以 Google C++ 风格为基础，关闭短函数、lambda、条件、循环和代码块的单行压缩，函数及类定义之间自动留空行。控制语句使用大括号，函数内部按逻辑步骤用空行分段。统一使用 clang-format 14，避免不同版本生成额外差异：

```bash
python3 scripts/format_cpp.py
python3 scripts/format_cpp.py --check
```

脚本覆盖项目 C++ 头文件、源文件与 CUDA 文件，排除 `third_party`、bundled ODE/Boost、LBFGS 和 NVIDIA `helper_math.h`。不会改动 Python、YAML 或 CMake。轨迹测试 `trajectory_kinematics` 使用有限差分校验七种轨迹的位置/速度/加速度一致性；集成脚本覆盖七种轨迹、CSV 优先级与非法轨迹参数拒绝。
