# UAV Predict Sim

面向无人机轨迹预测研究的无 ROS 数值仿真项目。计划参考高飞团队（ZJU FAST Lab）的 EGO-Planner 等开源规划器中的 `uav_simulator`，抽离动力学、控制与仿真逻辑，在独立 C++ 程序中生成轨迹、运行预测算法并进行定量评测。

使用 **matplotlib-cpp** 输出实验图表，使用 **Pangolin** 展示三维场景、无人机运动与预测轨迹。核心仿真不依赖 ROS、catkin、ROS 消息或 ROS 时间。

## 当前状态

已实现首个可运行版本：固定步长质点仿真、限幅 PD 控制、悬停/直线/圆周参考轨迹、带噪位置观测、CV/CA 预测、时间对齐评测和 CSV 输出。可选接入 matplotlib-cpp 生成 PNG，以及 Pangolin 三维回放。

**当前动力学是自行实现的重力补偿质点模型，尚未移植 EGO-Planner simulator**；没有姿态、角速度、电机、空气阻力或碰撞模型。它用于先验证预测与评测流程，不能据此评价真实四旋翼控制性能。首次获取上游源码时遇到 TLS 下载失败，后续将独立完成刚体动力学适配。

## 构建与运行

核心依赖为 C++17 编译器、CMake 3.16+ 和 Eigen 3.3+。本机已验证 GCC 11.4、CMake 3.22.1、Eigen 3.4.0。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
ctest --test-dir build --output-on-failure
./build/uav_sim --scene circle --output outputs/circle-001
```

每次运行使用新的输出目录；程序拒绝覆盖已有目录。`--help` 查看所有参数。默认仿真 12 s、积分步长 0.01 s、观测周期 0.1 s、预测周期 0.2 s、预测时域 2 s、历史窗口 10 个位置样本、位置噪声标准差 0.02 m、种子 42。

```bash
# 无噪声直线基线；CV 和 CA 的误差应接近浮点精度
./build/uav_sim --scene line --noise 0 --output outputs/line-001

# 悬停
./build/uav_sim --scene hover --noise 0 --output outputs/hover-001
```

时间参数必须是积分步长的整数倍，预测周期和预测时域还须是观测周期的整数倍。历史窗口至少 3 个样本，仿真时长须容纳完整历史窗口和至少一次完整预测。状态初始化为参考轨迹在零时刻的位置及速度。

### 图表与三维显示

matplotlib-cpp 已固定版本并存入 `third_party/`，其来源、许可证和兼容性修改见 [第三方依赖记录](third_party/README.md)。绘图另需 Python 开发库、NumPy 和 Matplotlib；本机已验证 Python 3.10.12、NumPy 1.21.5 和 Matplotlib 3.5.1。Pangolin 使用本机安装的 CMake package。

```bash
cmake -S . -B build-visual -DCMAKE_BUILD_TYPE=Release \
  -DUAV_ENABLE_PLOTTING=ON -DUAV_ENABLE_VIEWER=ON \
  -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build build-visual -j2
ctest --test-dir build-visual --output-on-failure

# Agg 后端输出 PNG，不需要窗口；缓存目录需可写
MPLCONFIGDIR=/tmp/uav-predict-matplotlib \
  ./build-visual/uav_sim --plot --output outputs/circle-plot-001

# 在有 OpenGL 显示环境的桌面终端中运行
./build-visual/uav_sim --view --output outputs/circle-view-001
```

两个构建选项都默认关闭，也可以只开启其中一个。若 Pangolin 不在 CMake 默认搜索路径，设置 `-DPangolin_DIR=/path/to/lib/cmake/Pangolin`。`--plot` 和 `--view` 可以组合使用，禁用相应模块时程序会明确报错。

Pangolin 中可用鼠标调整视角，使用 `Play` 暂停/继续、`Sample` 拖动回放位置，使用 `Observations`、`CV`、`CA` 切换显示。黑线是真值历史，绿点是观测，蓝/橙线是最近一次已评测的 CV/CA 预测，红色十字仅标记当前位置，不表达姿态。预测线显示其原始完整时域，失效后隐藏。窗口关闭后程序退出。

PNG 生成已实际验证；Pangolin 已编译、链接通过，当前开发会话没有显示环境，尚未验证窗口交互。

### 输出与复现

| 文件 | 内容 |
| --- | --- |
| `config.txt` | 模型、坐标约定、参数、随机种子、配置 CMake 时的 Git revision |
| `states.csv` | 每个积分步的时间、位置、速度真值 |
| `observations.csv` | 采样时刻及带噪位置 |
| `predictions.csv` | 方法、预测起点、目标时间、时距、预测/真值位置及欧氏距离误差 |
| `metrics.csv` | 每次预测的 ADE、FDE 与预测耗时（微秒） |
| `lead_metrics.csv` | 各预测时距的三维位置 RMSE 及样本数 |
| `summary.png` | 使用 `--plot` 时生成的 XY 轨迹与 RMSE 图 |

CV/CA 分别对历史位置做一次/二次最小二乘拟合，拟合局部时间以改善数值条件；不读取真值速度。所有预测只使用起点及之前的观测，未来真值只用于离线评测。仅评测拥有完整未来真值的预测起点，不截短末尾预测时域。控制器为 `a = 4 * position_error + 2.8 * velocity_error`，默认加速度模长限幅 5 m/s²；每个积分步内保持加速度不变并精确积分质点方程。

相同可执行文件、标准库和种子可复现轨迹及指标，计时结果不保证相同；不同标准库的正态分布实现可能不同。提交代码后重新执行 CMake 配置和构建，以刷新记录的 Git revision（有已跟踪的未提交改动时带 `dirty` 标记）。

CTest 覆盖解析积分、悬停、CV/CA 解析预测、时间对齐、随机种子、无效参数和完整预测时域；存在 Python 解释器时还检查 CSV、重新计算 ADE/FDE/RMSE、重复运行一致性及拒绝覆盖。开启绘图时同时检查 PNG 输出。

## 项目目标

- 复用成熟 UAV simulator 中可独立运行的动力学与控制模块，逐步移除 ROS 耦合。
- 支持真值状态生成、带噪观测、历史轨迹缓存及多步轨迹预测。
- 建立统一实验接口，对比不同预测方法在不同运动模式和噪声条件下的表现。
- 通过固定仿真步长、显式随机种子和配置记录，使实验可以复现。
- 将计算、数据记录和显示解耦，支持无窗口批量实验与交互回放。

首期以单机、离线数值实验为主；完整规划器、复杂传感器仿真、多机协同和实机通信按实验需求逐步引入。

## 技术方案

| 模块 | 拟采用方案 | 职责 |
| --- | --- | --- |
| 工程与计算 | C++17、CMake、Eigen | 构建、状态表达与数值计算 |
| UAV 仿真 | 从上游 `uav_simulator` 评估并抽离 | 动力学积分、控制响应、状态真值 |
| 预测算法 | 统一 C++ 接口 | 输入历史观测，输出未来时间点的状态预测 |
| 实验记录 | CSV 与配置/元信息文件 | 保存真值、观测、预测、指标与运行参数 |
| 数据绘图 | matplotlib-cpp | 轨迹、误差、耗时和统计图表 |
| 三维显示 | Pangolin / OpenGL | 场景、机体姿态、历史与预测轨迹回放 |

matplotlib-cpp 是 Python Matplotlib 的 C++ 封装，仍需 Python 开发头文件、可链接的 Python 库及 Matplotlib；使用 NumPy 数据接口时也需 NumPy。无 ROS 不代表无 Python。Pangolin 用于交互显示，需要可用的窗口及 OpenGL 环境。

绘图和可视化已作为可选构建模块，核心仿真与批量评测可以在没有显示环境的机器上运行。

## 模块与数据流

```text
实验配置 / 随机种子
        |
参考轨迹 -> 控制器 -> UAV 动力学 -> 真值状态
                                      |
                                 观测采样 / 噪声
                                      |
                                  历史观测窗口
                                      |
                                   预测算法
                                      |
                        预测结果 + 对齐的未来真值
                                      |
                              评测 / 数据记录
                               /            \
                    matplotlib-cpp       Pangolin
                       实验图表          三维回放
```

仿真时钟由固定步长推进，显示帧率不决定积分步长。动力学积分频率、观测频率和预测频率分别配置；预测只读取当前及过去的观测，未来真值仅用于评测。

统一约定 SI 单位，内部时间以仿真起点的秒数表示。接入动力学模型时明确世界坐标系、机体坐标系、重力方向、旋转方向及四元数分量顺序，并在适配边界完成转换，避免不同上游约定混用。

## 规划目录

以下目录将在对应模块实现时创建：

```text
uav_predict_sim/
├── CMakeLists.txt
├── cmake/                   # 依赖查找与构建选项
├── include/uav_predict_sim/  # 公共接口与数据类型
├── src/
│   ├── simulation/          # 动力学、控制器、仿真时钟
│   ├── prediction/          # 预测接口与算法
│   ├── evaluation/          # 时间对齐与误差指标
│   ├── plotting/            # matplotlib-cpp 绘图
│   └── visualization/       # Pangolin 显示
├── apps/                    # 仿真、评测与回放入口
├── configs/                 # 可追踪的实验配置
├── tests/                   # 数值正确性与回归检查
├── third_party/             # 固定版本的第三方代码及许可
├── docs/                    # 模型、接口与实验说明
└── outputs/                 # 本地生成结果，不纳入 Git
```

## 仿真器复用原则

优先评估 [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner) 中的仿真代码，并按需求对照 [EGO-Swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) 等项目。最终选择、源码路径和版本在实际接入时记录。

1. 先梳理动力学、积分器、控制器及其依赖，区分算法实现与 ROS 包装。
2. 用普通 C++ 数据结构、函数调用和仿真时钟替代消息、节点、参数服务器及回调。
3. 首次移植尽量保持原始方程和参数，以悬停、匀速及转弯等工况检查数值行为，再逐步修改模型。
4. 记录每份引入代码的上游地址、commit、原文件路径、许可证与本地改动，保留版权声明。

EGO-Planner 仓库标注为 GPL-3.0；引入前应核对具体文件及其依赖的许可证，再确定本仓库的分发许可。当前尚未导入 EGO-Planner 代码，也尚未指定本项目许可证。已引入的 matplotlib-cpp 保留其原始许可证。

## 实验与评测规划

首先实现匀速（CV）和匀加速（CA）预测基线，再按研究需求扩展其他方法。场景从直线、圆周、加减速与转弯开始，逐步加入测量噪声、采样变化和外部扰动。

每次实验计划记录：

- 实验配置、随机种子、代码 commit、模型参数及算法参数。
- 仿真步长、观测周期、历史窗口、预测周期及预测时域。
- 带时间戳的真值、观测、预测轨迹及每次预测的起始时刻。
- 平均位移误差（ADE）、最终位移误差（FDE）、分预测时距的位置 RMSE 和单次预测耗时。

指标在相同时间戳和相同坐标系下计算，并注明距离单位、有效样本数和无效样本处理规则。观测噪声与模型扰动分开配置，避免混淆误差来源。跨多次实验报告统计结果时使用相同场景及种子集合。

图表计划包括三维轨迹、位置/速度时序、误差随预测时距的变化以及耗时分布。Pangolin 计划支持暂停、回放，以及真值、观测和预测轨迹的独立显示。

## 开发顺序

- [x] 编写项目 README，初始化 Git。
- [x] 建立最小 CMake 工程、状态类型和固定步长仿真入口。
- [ ] 选定并记录上游 simulator 版本，完成无 ROS 动力学与控制适配。
- [x] 基于质点模型实现简单场景与 CSV 记录，验证悬停和基础运动。
- [x] 接入 CV / CA 基线、时间对齐和误差评测。
- [x] 接入 matplotlib-cpp，生成首组对比图。
- [ ] Pangolin 三维回放已实现并编译，待桌面环境交互验证。
- [ ] 扩展场景、扰动和预测算法，形成可复现实验集。

已验证的构建与实验命令见前文。下一阶段优先引入上游刚体动力学与控制模块，并补充姿态约定及模型回归检查。

## Git 工作约定

- 小步提交，每次提交围绕一个明确改动，避免混合依赖导入、重构和算法修改。
- 提交信息使用 `docs:`、`build:`、`feat:`、`fix:`、`test:`、`chore:` 等前缀。
- 每个可运行阶段执行相应构建或数值检查，并记录验证结果。
- 版本化实验配置与必要的小型参考数据；构建产物、大批量原始数据、图表及本机环境文件不进入仓库。
- 第三方源码单独引入并固定版本，移植改动使用后续提交记录。

## 参考项目

- [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner)：规划器与 UAV simulator 参考来源。
- [EGO-Swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm)：相关规划与仿真实现参考。
- [matplotlib-cpp](https://github.com/lava/matplotlib-cpp)：C++ 数据绘图接口。
- [Pangolin](https://github.com/stevenlovegrove/Pangolin)：OpenGL 显示与交互库。
