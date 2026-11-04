# EGO-Planner-Swarm UAV Simulator 使用说明

本文针对本工作区 `src/ego-planner-swarm` 的现有代码，整理时间为 2026-11-03。目标是先验证已有仿真器，再接入单机规划；暂不扩展预测算法或多机功能。

**验证范围：**已核对源码、launch、消息和构建配置，并运行 `colcon list` 确认包可被发现。当前环境为 Ubuntu 22.04、ROS 2 Humble；尚未编译或启动仿真，因此下列步骤是按当前代码整理的操作流程，不代表已经完成运行验收。

## 1. 仿真器包含什么

这套 simulator 是 ROS 节点组成的轻量四旋翼仿真系统，使用自己的动力学代码，无需先启动 Gazebo、PX4 或 MAVROS。

| 目录 / ROS 包 | 作用 |
| --- | --- |
| `uav_simulator/so3_quadrotor_simulator` | 四旋翼动力学、里程计与 IMU 输出 |
| `uav_simulator/so3_control` | 将位置、速度、加速度和偏航指令转换为 SO3 控制指令 |
| `uav_simulator/fake_drone` / `poscmd_2_odom` | 简化模式，将位置指令转换为里程计；目录名不等于包名 |
| `uav_simulator/map_generator` | 随机障碍物点云地图 |
| `uav_simulator/mockamap` | 另一种地图生成器 |
| `uav_simulator/local_sensing` | 根据地图与机体位置生成局部感知点云 |
| `uav_simulator/Utils/odom_visualization` | 机体模型、轨迹和相关可视化 |
| `uav_simulator/Utils/quadrotor_msgs` | PositionCommand、SO3Command 等消息 |
| `planner/plan_manage` / `ego_planner` | 规划器、轨迹服务器及集成 launch |

两种模式的数据链路：

```text
use_dynamic:=True
规划器 → B 样条 → traj_server → PositionCommand → SO3 控制器
                                                     ↓
                                              SO3Command
                                                     ↓
                                               四旋翼动力学
                                                     ↓
                                                  Odometry
                                                     ↓
                         规划器 ← 局部点云 ← 地图 + local_sensing

use_dynamic:=False
规划器 → traj_server → PositionCommand → poscmd_2_odom → Odometry
```

简化模式适合先排查规划和话题连接；动力学模式用于观察控制跟踪过程。地图主要服务于感知与规划，不应把障碍物点云理解成物理碰撞引擎。

## 2. 环境与编译

以下路径以本机为准；换机器时修改工作区路径。所有运行终端应使用相同的 ROS 环境和 DDS 设置。

### 2.1 检查基础环境

```bash
cd /home/ld/uav_predict_sim
source /opt/ros/humble/setup.bash
printenv ROS_DISTRO
colcon list --base-paths src
```

应能看到 `so3_quadrotor_simulator`、`so3_control`、`poscmd_2_odom`、`ego_planner` 等包。

原仓库 Readme 提醒默认 FastDDS 可能卡顿，并建议使用 CycloneDDS。可先检查对应包：

```bash
ros2 pkg prefix rmw_cyclonedds_cpp
```

若未安装，按需执行：

```bash
sudo apt update
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

每个运行终端显式设置即可，不必先修改 `.bashrc`：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### 2.2 安装依赖

源码使用 Eigen、Armadillo、PCL/VTK，以及 ROS 消息、组件、可视化相关包。建议先通过 rosdep 解析已声明依赖：

```bash
cd /home/ld/uav_predict_sim
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

若系统从未初始化 rosdep，先执行一次 `sudo rosdep init`。部分 package.xml 声明可能不完整，例如动力学 CMake 显式要求 Armadillo，但 package.xml 未声明它；以构建日志中的缺失项为准补齐，常见系统库包括：

```bash
sudo apt install libeigen3-dev libarmadillo-dev libpcl-dev
```

当前 `local_sensing/CMakeLists.txt` 中 `ENABLE_CUDA=false`，默认构建 CPU 点云版本，首轮无需安装 CUDA。不要仅为跑通现有默认路径切换 GPU 分支。

### 2.3 分阶段编译

先编译独立动力学示例所需的包：

```bash
cd /home/ld/uav_predict_sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-up-to so3_quadrotor_simulator so3_control odom_visualization \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

之后做单机规划时，再编译集成路径：

```bash
colcon build --symlink-install \
  --packages-up-to ego_planner map_generator mockamap local_sensing \
                    poscmd_2_odom so3_quadrotor_simulator so3_control odom_visualization \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

这两条命令显式列出运行时仿真包，避免仅编译 `ego_planner` 时漏掉未完整声明的运行依赖。若内存不足，可加 `--executor sequential`；单包编译仍耗内存时再限制构建并发。

### 2.4 每个新终端的公共准备

后续各步骤的新终端，都先执行：

```bash
cd /home/ld/uav_predict_sim
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## 3. 第一步：独立运行动力学仿真器

先验证指令、控制器、动力学、里程计闭环。这一步不包含规划器、地图或局部感知。

### 3.1 启动仿真

终端 A 完成公共准备后执行：

```bash
ros2 launch so3_quadrotor_simulator simulator_example.launch.py \
  init_x:=0.0 init_y:=0.0 init_z:=1.0
```

该 launch 同时启动动力学节点、SO3 控制组件容器、里程计可视化与 RViz。原默认初始位置为 `(-4, 0, 4)`，这里显式改成 `(0, 0, 1)`，便于进行小幅位置指令验证。

### 3.2 发送一个固定位置指令

终端 B 完成公共准备后执行：

```bash
ros2 topic pub --rate 20 /position_cmd quadrotor_msgs/msg/PositionCommand \
  '{position: {x: 1.0, y: 0.0, z: 1.0}, velocity: {x: 0.0, y: 0.0, z: 0.0}, acceleration: {x: 0.0, y: 0.0, z: 0.0}, yaw: 0.0, yaw_dot: 0.0}'
```

这会持续发送位置目标。消息中未填写的字段使用默认值；`kx/kv` 全零时控制器保留其配置增益。

### 3.3 验证输出

终端 C 完成公共准备后执行，`hz` 命令观察数秒后用 Ctrl+C 退出：

```bash
ros2 component list
ros2 topic info /position_cmd --verbose
ros2 topic hz /sim/odom
ros2 topic echo /sim/odom --once
ros2 topic echo /so3_cmd --once
```

验收条件：

- SO3 控制组件成功加载，节点持续运行。
- `/position_cmd` 有订阅者，`/so3_cmd` 持续产生控制消息。
- `/sim/odom` 有数据；配置目标为约 100 Hz，实际频率受机器负载影响。
- 里程计位置逐步趋近 `(1, 0, 1)`，没有 NaN、发散或持续退出重启。

独立示例的话题为 `/sim/odom`、`/sim/imu`、`/position_cmd`、`/so3_cmd`。不要与集成模式的 `drone_0_...` 话题混用。

### 3.4 RViz 显示与自带示例

RViz 使用 `world` 作为 Fixed Frame。随附配置不一定已经显示机体；必要时增加 Marker 显示 `/robot`、Path 显示 `/path`。本步骤没有地图，配置中的 `/mock_map` 没有数据属于预期现象。若出现 TF 错误，注意动力学原始里程计使用 `/world`，可视化部分使用 `world`，应结合实际 RViz Status 检查。

仓库另有可执行示例：

```bash
ros2 run so3_control control_example
```

运行前先停止终端 B 的发布命令，避免两个源同时发 `/position_cmd`。该程序以约 100 Hz 循环切换位置、速度、加速度控制，每段约 5 秒；并非固定悬停测试。首轮优先使用上面的固定位置命令。

结束本步骤时，停止指令发布终端和仿真 launch，再启动下面的集成模式。

## 4. 第二步：单机规划 + 简化仿真

完成第 2.3 节集成路径编译后，终端 A 完成公共准备并启动 RViz：

```bash
ros2 launch ego_planner rviz.launch.py
```

终端 B 完成公共准备并启动单机：

```bash
ros2 launch ego_planner single_run_in_sim.launch.py \
  use_mockamap:=False use_dynamic:=False
```

当前入口会启动地图、规划器、轨迹服务器，以及 `simulator.launch.py` 中的简化无人机、可视化和局部感知节点。

当前写在 launch 中的设置：

| 设置 | 当前值 / 含义 |
| --- | --- |
| `drone_id` | 默认 0 |
| 初始位置 | `(-15.0, 0.0, 0.1)`，在单机 launch 内写定 |
| `flight_type` | 2，预设航点模式 |
| `point_num` | 4，使用 point0 至 point3 |
| 航点 | `(15,0,1)` → `(-15,0,1)` → `(15,0,1)` → `(-15,0,1)` |
| 最大速度 / 加速度 | 2.0 m/s、6.0 m/s² |
| 规划范围参数 | 默认 `50 × 25 × 2` m |
| Random Forest 地图 | 节点内另行写定 `26 × 20 × 3` m，不随上述范围参数同步变化 |

当前配置 `fsm/realworld_experiment=False`，FSM 将初始 `have_trigger_` 设为 true；预设航点模式在收到里程计后可继续运行。不能仅凭 “Waiting for trigger from [n3ctrl] from RC” 日志判断必须连接遥控器。

默认也不是 RViz 点选目标模式。若以后需要手动目标，应修改单机 launch 传给 `advanced_param.launch.py` 的 `flight_type` 为 1，并使用 `/move_base_simple/goal`（PoseStamped）。当前单机入口内将 flight_type 写成固定值，不应假设添加命令行 `flight_type:=1` 就一定生效。

### 4.1 检查完整链路

```bash
ros2 node list
ros2 topic echo /map_generator/global_cloud --once
ros2 topic echo /drone_0_visual_slam/odom --once
ros2 topic echo /drone_0_pcl_render_node/cloud --once
ros2 topic echo /drone_0_planning/bspline --once
ros2 topic echo /drone_0_planning/pos_cmd --once
```

若 `echo --once` 一直等待，可用 Ctrl+C 退出，再用 `ros2 topic info <话题> --verbose` 检查发布订阅情况。轨迹消息通常在规划更新时才发布，不一定持续高频输出。

验收条件是全局地图、局部点云、里程计均有数据，规划器产生轨迹和位置指令，机体随位置指令移动；仅看到 RViz 窗口不算闭环跑通。

## 5. 第三步：单机规划 + 动力学仿真

停止上一节的单机 launch，保留或重新启动 RViz，然后执行：

```bash
ros2 launch ego_planner single_run_in_sim.launch.py \
  use_mockamap:=False use_dynamic:=True
```

此时应使用 SO3 控制器和动力学节点替代 `poscmd_2_odom`。额外检查：

```bash
ros2 component list
ros2 topic echo /drone_0_so3_cmd --once
ros2 topic hz /drone_0_visual_slam/odom
```

验收时同时观察期望位置与实际里程计的跟踪情况。若简化模式正常、动力学模式异常，重点检查组件加载、控制话题、控制增益和初始高度。单机入口初始高度写定为 0.1 m；若需要调整，修改对应 launch 后重新加载工作区构建结果。

当前 `simulator.launch.py` 没有像独立示例那样重映射 IMU；默认无命名空间时动力学 IMU 位于 `/imu`。扩展多机时需要单独核对这类未按 drone_id 隔离的话题。

## 6. 关键接口与参数

集成模式下，`drone_id=0`、默认 odometry_topic 时：

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/map_generator/global_cloud` | `sensor_msgs/msg/PointCloud2` | 全局障碍物点云 |
| `/drone_0_pcl_render_node/cloud` | `sensor_msgs/msg/PointCloud2` | 局部感知点云 |
| `/drone_0_visual_slam/odom` | `nav_msgs/msg/Odometry` | 仿真实际状态 |
| `/drone_0_planning/bspline` | `traj_utils/msg/Bspline` | 规划轨迹 |
| `/drone_0_planning/pos_cmd` | `quadrotor_msgs/msg/PositionCommand` | 轨迹服务器的参考指令 |
| `/drone_0_so3_cmd` | `quadrotor_msgs/msg/SO3Command` | 动力学模式控制指令 |
| `/drone_0_vis/robot` | `visualization_msgs/msg/Marker` | 机体显示 |
| `/drone_0_vis/path` | `nav_msgs/msg/Path` | 实际运动轨迹显示 |

位置单位为 m，速度 m/s，加速度 m/s²，yaw 为 rad，yaw_dot 为 rad/s。PositionCommand 还包含增益与轨迹状态字段；完整定义见 `quadrotor_msgs/msg/PositionCommand.msg`。

| 参数 / 位置 | 当前默认值或注意点 |
| --- | --- |
| 单机入口 `use_dynamic` | False |
| 子 launch `simulator.launch.py` 的 `use_dynamic` | True；与单机入口不同 |
| `use_mockamap` | 单机入口 False；True 切换 mockamap |
| 独立示例 `init_x/init_y/init_z` | `-4.0 / 0.0 / 4.0` |
| 集成子 launch `init_x_/init_y_/init_z_` | 参数名带尾部下划线 |
| 动力学 `rate/simulation` | 1000.0 Hz，数值积分配置 |
| 动力学 `rate/odom` | 100.0 Hz |
| SO3 控制器 mass | launch 设置 0.98 kg |
| 集成局部感知 | sensing_horizon=5.0 m，sensing_rate=30.0 Hz |

查看入口参数：

```bash
ros2 launch so3_quadrotor_simulator simulator_example.launch.py --show-args
ros2 launch ego_planner single_run_in_sim.launch.py --show-args
```

**CPU 默认分支输出的是点云。**虽然集成 launch 中写有 depth 的重映射及 camera.yaml，默认编译的 `pointcloud_render_node.cpp` 仅创建局部点云发布器。不能用深度图话题是否有输出作为默认模式的验收标准。

## 7. 当前代码的已知使用陷阱

1. **单独启动集成子 launch 没有地图。**`simulator.launch.py` 定义了地图节点，但将它们加入 LaunchDescription 的两行被注释。地图由 `single_run_in_sim.launch.py` 提供；改 `p_num/c_num/use_mockamap` 并不会让该子 launch 自己生成地图。
2. **另一入口 `run_in_sim.launch.py` 存在参数名不一致。**它向 simulator 传入的是 `use_dynamic_cmd`，子 launch 读取的是 `use_dynamic`。这属于需要排查的实现问题；首轮按本文使用 `single_run_in_sim.launch.py`。
3. **修改地图范围不一定改变实际障碍物范围。**单机入口的 Random Forest 尺寸与数量写在节点参数内；`obj_num` 也不能直接理解为静态障碍物数量。
4. **改 odometry_topic 要同时核对所有 remap。**动力学和可视化部分固定使用 `visual_slam/odom`，感知部分则拼接配置值。首轮保留默认值，避免反馈链路断开。
5. **相同 drone_id 的多个实例可能互相干扰。**先停止上一轮 launch，再切换模式；不要同时运行两个单机实例。
6. **源码已有不代表环境已安装。**`ros2 pkg prefix` 查不到包时，先检查构建结果和 `source install/setup.bash`，不要直接反复重装 ROS。

## 8. 按症状排查

| 现象 | 先检查 |
| --- | --- |
| Package not found | 是否编译成功；当前终端是否 source 了本工作区 install/setup.bash |
| 编译找不到 Armadillo/PCL/VTK | 首个 CMake 错误对应的系统库；package.xml 是否遗漏声明 |
| SO3 组件加载失败 | so3_control 是否构建并安装；组件插件注册和 YAML 路径；容器错误日志 |
| 没有里程计 | 仿真进程是否存活、模式是否正确、话题名是否匹配 |
| 有里程计但不运动 | 是否有 PositionCommand；是否产生 SO3Command；规划器是否已产生轨迹 |
| 没有局部点云 | 是否收到全局地图和里程计；感知范围内是否有障碍物 |
| 没有深度图 | 先确认是否为默认 CPU 点云分支，见第 6 节 |
| RViz 空白或显示报错 | Fixed Frame、显示话题、TF、Marker 资源和 RViz Status |
| 节点存在但发现不了消息 | 各终端 DDS 与 ROS_DOMAIN_ID 是否一致；话题 QoS 是否匹配 |
| 运行明显卡顿 | 原仓库建议的 CycloneDDS 是否启用；CPU/内存负载；地图密度及 RViz 显示数量 |

如需保存单机运行数据，可以在运行期间执行：

```bash
cd /home/ld/uav_predict_sim
ros2 bag record -o /tmp/uav_single_run \
  /drone_0_visual_slam/odom /drone_0_planning/pos_cmd \
  /drone_0_so3_cmd /drone_0_planning/bspline
```

每次录制使用新的输出目录；简化模式没有 SO3 指令是正常的。Ctrl+C 正常停止录制。

## 9. 本阶段完成标准与源码索引

按顺序记录三项结果即可：

- [ ] 独立动力学示例：固定位置指令能让实际位置收敛。
- [ ] 单机简化模式：地图、感知、规划与位置执行链路有数据且机体移动。
- [ ] 单机动力学模式：SO3 控制组件正常，实际机体能跟踪规划指令。

三项都完成后，再考虑多机、预测模块或替换控制接口。

关键源码（路径相对于 `/home/ld/uav_predict_sim/src/ego-planner-swarm`）：

- `Readme.md`：仓库原始启动说明与 DDS 建议。
- `src/uav_simulator/so3_quadrotor_simulator/launch/simulator_example.launch.py`：独立动力学入口。
- `src/uav_simulator/so3_quadrotor_simulator/src/quadrotor_simulator_so3.cpp`：动力学接口和频率配置。
- `src/uav_simulator/so3_control/src/so3_control_component.cpp`：位置指令处理和 SO3 输出。
- `src/uav_simulator/so3_control/src/control_example.cpp`：三种控制示例。
- `src/planner/plan_manage/launch/single_run_in_sim.launch.py`：单机集成入口。
- `src/planner/plan_manage/launch/simulator.launch.py`：仿真、控制、感知和可视化组合。
- `src/planner/plan_manage/launch/advanced_param.launch.py`：规划器配置。
- `src/planner/plan_manage/src/ego_replan_fsm.cpp`：预设航点与触发逻辑。
- `src/uav_simulator/local_sensing/CMakeLists.txt`：CPU / CUDA 分支选择。
