# 轨迹预测数值仿真

本项目生成已知运动轨迹，通过引用的 `poscmd_2_odom` 运动学仿真器输出里程计，再加入观测噪声并执行 IMM 轨迹预测。Pangolin 实时显示真实轨迹、观测、滤波估计和未来预测，结束后导出误差统计、曲线图和录像。

## 编译

依赖 Ubuntu 22.04 / ROS 2 Humble、Eigen3、Pangolin、Python3 开发库、系统 Matplotlib 和 FFmpeg。Pangolin 需提前安装；matplotlib-cpp 已随 `src/visualize/third_party` 提供。

在工作区根目录执行，只构建数值仿真所需的包：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-up-to predictor prediction_visualize poscmd_2_odom \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
```

若 CMake 找不到 Pangolin，在构建参数中添加 `-DPangolin_DIR=/path/to/lib/cmake/Pangolin`。

## 启动一次仿真

先修改 `src/predictor/config/simulation.yaml` 中的 `output_dir`，为本次运行选择新的目录，然后执行：

```bash
scripts/run_simulation.sh
```

脚本自动加载 ROS 和本工作区环境。默认预热约 2 秒，然后运行 30 秒三维八字轨迹。Pangolin 左侧显示真实轨迹、含噪观测、滤波估计和未来预测，右侧显示位置/预测误差、位置协方差对角线和模型概率。鼠标可旋转、平移、缩放三维视角。

仿真结束后窗口自动关闭，脚本生成统计、曲线图与 MP4。提前关闭窗口会中止本轮仿真并报告失败。图形运行需要有效的 `DISPLAY` 和 OpenGL 环境。

使用自己的数值配置和显示配置：

```bash
scripts/run_simulation.sh \
  --config /absolute/path/to/simulation.yaml \
  --visualization-config /absolute/path/to/visualization.yaml
```

无图形环境时运行：

```bash
scripts/run_simulation.sh --config /absolute/path/to/simulation.yaml --headless
```

`--headless` 保留数值仿真、CSV、统计和离线曲线图，不打开窗口或生成视频。运行器拒绝覆盖已有数据；每轮修改 `output_dir`。使用固定 `noise_seed` 可固定噪声随机序列，实际采样时间仍受 ROS 调度影响。

## 两份配置文件

| 文件 | 内容 |
| --- | --- |
| `config/simulation.yaml` | 时长、轨迹、噪声、预测模型、输出路径、显示/录像开关、窗口尺寸与帧率 |
| `config/visualization.yaml` | Pangolin 浅色/深色主题、线宽、点大小、网格、相机、曲线纵轴范围和显示开关 |

默认使用上述两份文件。`--config` 和 `--visualization-config` 可以分别替换它们，省略显示配置参数时继续加载默认的 `visualization.yaml`。显示配置采用 `prediction_viewer.ros__parameters` 结构；同名参数优先使用显示配置。修改显示参数后重新启动生效，不需要重新编译。

`output_dir`、`input_csv` 和 `ffmpeg_path` 的相对路径以数值配置所在目录为基准；CSV、日志、图像、录像帧等输出路径以 `output_dir` 为基准。支持绝对路径和含空格路径。每轮的 `resolved.yaml` 保存解析后的数值参数与显示参数。

### 切换浅色/深色主题

在 `config/visualization.yaml` 中只需修改一个参数：

```yaml
prediction_viewer:
  ros__parameters:
    theme: light   # light 为浅色，dark 为深色
```

背景、文字、轨迹、误差曲线与四旋翼配色会统一切换，无需配置 RGB。修改后重新启动生效。录像采用同一主题；离线 `summary.png` 使用独立的 Matplotlib 样式。

### 调整窗口大小

运行时可以拖动窗口边框或最大化，三维视图、曲线区、相机投影和文字布局会跟随实际窗口大小调整。相机的鼠标旋转和平移结果会保留。

初始窗口大小在 `simulation.yaml` 中设置，单位为像素：

```yaml
    window_width: 1280
    window_height: 800
```

宽高为正整数，允许奇数。录像帧保留采集时的实际窗口尺寸；最终 MP4 固定为初始窗口尺寸（奇数向上补齐为偶数），缩放后按比例完整放入画布，必要时补边，避免裁切与视频尺寸变化。窗口调整不会改变数值仿真结果。

常用字段：

| 参数 | 作用 |
| --- | --- |
| `theme` | `light` / `dark`，统一切换整个实时面板的配色 |
| `trajectory_line_width` | 轨迹线宽，像素 |
| `observation_point_size`、`estimate_point_size` | 观测点和当前位置点的大小，像素 |
| `show_grid`、`show_legend`、`show_uncertainty` | 网格、标题/图例/时间、三轴 2σ 线段开关 |
| `grid_extent`、`grid_spacing` | 地面网格半径和间隔，m |
| `camera_eye`、`camera_target` | 初始相机位置与观察目标，世界坐标系，m，Z 轴向上 |
| `camera_focal_length` | 初始焦距，像素，越大视角越窄 |
| `error_plot_max`、`covariance_plot_max` | 初始误差/协方差纵轴上限，m / m² |
| `max_track_points` | 窗口保留的最近轨迹点数，不截断 CSV |

## 查看四旋翼姿态

启动数值仿真后，Pangolin 左侧默认显示随位置和姿态运动的四旋翼。模型资源位于 `src/visualize/meshes/`：原始资源为 `hummingbird.mesh`，运行时读取转换后的 `hummingbird.ply`。详见 [模型来源与转换说明](../visualize/meshes/README.md)。

`visualization.yaml` 中可调整：

```yaml
    show_drone: true
    drone_pose_source: truth
    drone_scale: 2.0
    show_drone_axes: true
    drone_axis_length: 0.45
```

默认 `truth` 使用仿真里程计的位置和四元数；`estimate` 使用滤波输出，观察估计姿态时请保持数值配置中的 `use_attitude: true`。不使用轨迹切线或速度方向代替姿态。仿真器根据加速度和偏航构造运动学姿态，这里显示该姿态，未引入额外动力学。

机头橙色箭头沿机体系 +X，三根坐标轴 X 红、Y 绿、Z 蓝，随机体一起旋转。`drone_scale` 仅改变显示倍率，不影响位置、轨迹或滤波计算。机体颜色随主题切换，通过 `show_drone: false` 隐藏模型。若要替换模型，`drone_mesh_file` 可指定包含三角面的 PLY；相对路径以显示配置所在目录为基准，顶点应使用 m 并与机体系对齐。空字符串使用内置模型。

修改 `odometry_topic` 时，启动流程会将四旋翼的真值订阅同步重映射。模型也包含在面板录像中。

## 选择轨迹

修改 `simulation.yaml` 中的 `trajectory` 与对应的 `trajectory_` 参数。位置、速度、加速度均使用世界坐标系，单位为 m、s、rad；向量按 x/y/z 排列。

| trajectory | 运动 | 对应参数（均带 `trajectory_` 前缀） |
| --- | --- | --- |
| `hover` | 固定位置 | `origin` |
| `straight` | 匀速直线 | `origin`、`velocity` |
| `acceleration` | 恒加速度 | `origin`、`velocity`、`acceleration` |
| `circle` | 水平圆周 | `origin`（圆心）、`radius`、`angular_speed`、`circle_phase` |
| `helix` | 圆周加匀速升降 | 圆周参数、`vertical_speed` |
| `figure8` | 三维八字 | `origin`、`amplitude`、`frequency`、`phase`；y 频率固定为 x 的两倍 |
| `sinusoid` | 三轴独立正弦 | `origin`、`amplitude`、`frequency`、`phase` |

例如半径 3 m、每 20 秒一圈、以 0.1 m/s 上升的螺旋：

```yaml
    output_dir: ../../../outputs/helix_trial_01
    trajectory: helix
    trajectory_origin: [0.0, 0.0, 2.0]
    trajectory_radius: 3.0
    trajectory_angular_speed: 0.3141592653589793
    trajectory_vertical_speed: 0.1
```

圆周和螺旋从圆周上开始，初始相位由 `trajectory_circle_phase` 指定；负角速度反转方向，负升降速度下降。仿真器从所选轨迹 t=0 的位置初始化。指令包含解析的位置、速度和加速度。

`input_csv` 非空时使用 CSV 替代内置轨迹。表头为 `t,px,py,pz,vx,vy,vz,ax,ay,az`，时间从 0 开始、严格递增并覆盖整个 `duration`；采样之间逐列线性插值。请使用运动学一致的数据。

## 调整数值参数

- `duration`：运行时长，秒；`sample_rate`：轨迹指令频率，Hz，仿真器里程计为 100 Hz。
- `inject_noise`：是否注入观测噪声；`position_noise_std`、`velocity_noise_std` 为位置/速度噪声标准差。关闭噪声时仍保留正的标准差配置，供滤波器使用。
- `motion_models`：可选择 `[cv, ca]` 或加入 `body_velocity`。分别采用世界系匀速、恒加速度和恒机体系速度假设。
- `stay_probability`：默认模型转移矩阵的对角概率；也可通过 `model_transition` 与 `model_initial_probability` 指定转移矩阵和先验。
- `prediction_algorithm`：当前为 `imm`，所有算法通过统一基类创建。
- `prediction_step × prediction_steps`：预测总时域，默认 1 秒。
- `use_attitude`：是否使用姿态观测。本项目引用的仿真器将加速度写在 `twist.angular` 中，保持 `use_angular_velocity: false`。
- `show_viewer`、`record_video`：是否显示窗口、录制视频；`window_width/height` 为初始窗口宽高，必须为正整数，`video_fps` 为 1～120。

预测只使用当前及过去的含噪观测。仿真真值用于显示和误差评估。

## 查看结果

默认写入 `outputs/demo/`，各文件名可在数值配置中修改：

| 输出 | 内容 |
| --- | --- |
| `samples.csv` | 真值、观测、估计、位置标准差、模型概率及误差 |
| `predictions.csv` | 预测时域终点与按目标时间对齐的真值、误差 |
| `metrics.json` | 位置/速度/预测 RMSE、NEES、每轴 2σ 覆盖率 |
| `summary.png` | 离线轨迹、误差、协方差与概率曲线 |
| `simulation.mp4` | Pangolin 完整面板录像 |
| `frames/`、`frame_times.csv` | 原始录像帧与采集时间 |
| `resolved.yaml`、`launch.log` | 本次数值/显示配置快照及运行日志 |

末尾尚未到达目标时间的预测不计入误差；聚合滤波指标排除预热阶段。录像按实际帧时间编码，渲染变慢时仍保留实际播放时长。默认噪声参数是试验起点，NEES 与覆盖率用于检查协方差是否校准。

对已有结果重新生成统计、曲线和视频：

```bash
scripts/run_simulation.sh --config /absolute/path/to/simulation.yaml --finalize-only
# 原来是无图形运行时：
scripts/run_simulation.sh --config /absolute/path/to/simulation.yaml --finalize-only --headless
```

## 统一预测算法接口

所有算法通过 [core/predictor.hpp](core/predictor.hpp) 的 `predictor::Predictor` 基类调用，预测结果统一为三维位置序列，内部状态维度由各算法自行决定。ROS `prediction` 话题仅携带这些预测位置，姿态字段为单位四元数占位。当前估计姿态、协方差和概率保留为可选诊断，供仿真面板与评估使用。

新增算法、输入输出约定与 `PoseKF` 占位类的说明见 [统一接口说明](core/README.md)。
