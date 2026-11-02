# 来源、版本与本地差异

## EGO-Planner UAV simulator

来源：https://github.com/ZJU-FAST-Lab/ego-planner

固定参考 commit：`bfda51284c8c1b476043255a8145ef925a3778a5`。

本项目的刚体、执行器和控制实现基于以下文件学习、抽离和改写：

- `src/uav_simulator/so3_quadrotor_simulator/src/dynamics/Quadrotor.cpp`
- `src/uav_simulator/so3_quadrotor_simulator/include/quadrotor_simulator/Quadrotor.h`
- `src/uav_simulator/so3_quadrotor_simulator/src/quadrotor_simulator_so3.cpp` 的 `getControl()`
- `src/uav_simulator/so3_control/src/SO3Control.cpp`

上游代码由 ZJU FAST Lab 及其贡献者提供；原文件未附独立逐文件版权头。上游根 LICENSE（GPL-3.0）完整保存在本项目根 `LICENSE`，改写文件注明来源。本项目新代码亦按 GPL-3.0-only 提供；第三方库保留自身许可证。

改动：移除 ROS/消息/节点依赖；状态采用单位四元数与固定步长 RK4；初始化所有扰动；删除静默 NaN 清零和地板修正；共享飞行器参数；默认关闭阻力；保留 RPM 单位及 + 构型混控；暴露原始分配值和饱和状态。后续 SO(3) 控制扩展和回归结果记录于 `docs/validation.md`。

`tests/upstream/` 独立保存上述 `Quadrotor.h/.cpp` 和 `SO3Control.h/.cpp` 的原始回归副本：仅删除未使用的 ROS include、将内嵌旧版 odeint include 改为系统 Boost，以及添加来源说明。它们只编译进回归测试，控制器正式扩展不修改这些副本。短时动力学对照显式启用相同阻力并初始化外力/外力矩。

`ego-planner/` 仅是开发时的参考 checkout，不参与任何构建、测试或运行。所需模型和参数均已置于本项目源码；删除参考目录不会影响工程。

## matplotlib-cpp

来源：https://github.com/lava/matplotlib-cpp

参考 commit：`ef0383f1315d32e0156335e10b82e90b334f6d9f`。

自包含文件：`matplotlibcpp.h`、MIT `LICENSE` 和上游 `LICENSE.matplotlib`。头文件在该版本上将 `subplot()` 三个网格参数从 Python 浮点数改为整数，以兼容 Matplotlib 3.5.1。文件已实际复制到 `third_party/matplotlib-cpp/`，构建不读取其他工程或归档目录。

Pangolin、Eigen、yaml-cpp 和 Python/NumPy/Matplotlib 使用系统安装，不从 `last/` 或 `ego-planner/` 查找。
