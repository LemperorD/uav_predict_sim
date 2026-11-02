# 第三方依赖记录

## matplotlib-cpp

- 上游：https://github.com/lava/matplotlib-cpp
- 固定 commit：`ef0383f1315d32e0156335e10b82e90b334f6d9f`
- 文件：`matplotlibcpp.h`、`LICENSE`、`LICENSE.matplotlib`
- 来源：本机 `/home/ld/pure-nav/src/thirdparty/matplotlib-cpp` 中上述 commit 的干净 Git checkout；离线复制，未修改。
- 许可证：MIT，原声明和上游附带的 Matplotlib 许可说明一并保留。

本地改动：`subplot()` 的三个网格参数由 Python 浮点数改为整数，兼容本机 Matplotlib 3.5.1 对网格参数类型的检查。原始 header 的 SHA-256 为 `8a68ce477a9c46fc2994f7c03dee2a3b8526977dfa4f17eb1aa2011edb88c06e`。

## Pangolin

通过 CMake `find_package(Pangolin CONFIG REQUIRED)` 使用系统安装，不在仓库内复制。绘图和可视化均可关闭。

## EGO-Planner simulator

尚未引入。首次下载失败（TLS 连接中断）；当前 `src/core.cpp` 是本项目的质点测试模型，不是上游 simulator 的移植。后续引入须单独记录上游 commit、文件路径、许可证和移植改动。
