# 统一轨迹预测接口

所有应用层预测算法继承 `predictor::Predictor`，声明位于 `core/predictor.hpp`。该头文件只依赖 Eigen 和 C++ 标准库，不依赖 ROS、IMM 或某一种状态维度。

```cpp
class MyPredictor final : public predictor::Predictor
{
 public:
  void update(const predictor::Observation &observation) override;
  predictor::PositionTrajectory predict(double step, int count) const override;
  void reset() override;
 private:
  // 自行定义内部状态、模型、历史窗口或网络；基类没有固定状态向量。
};
```

## 输入

`Observation` 包含以秒为单位的时间戳与世界系三维位置。速度、加速度、姿态和角速度为可选观测，由具体算法决定是否需要。

- `timestamp` 必须有限，更新时严格递增；`reset()` 后重新开始计时。
- `position`、`velocity`、`acceleration` 使用同一个世界坐标系，单位为 m、m/s、m/s²。
- `orientation` 为机体系到世界系的四元数，`angular_velocity` 为机体系角速度，单位 rad/s。
- 无真值、噪声参数、协方差维度或模型权重等要求；仿真节点负责生成带噪观测。
- `Observation::validate()` 校验公共必需字段；具体算法负责校验所用的可选观测。

例如，当前 IMM 适配器需要速度；启用姿态观测时还需要有效的非零四元数。仅使用位置的算法可以忽略所有可选观测。

## 唯一的预测结果

`PositionTrajectory` 就是 `std::vector<Eigen::Vector3d>`。`predict(step, count)` 必须返回恰好 `count` 个有限 XYZ 点：

```text
位置[0]     对应最新观测时间 + step
位置[1]     对应最新观测时间 + 2*step
...
位置[count-1] 对应最新观测时间 + count*step
```

不包含当前位置，不返回速度、姿态、协方差或内部状态。`step` 必须为正且有限，`count` 为 1～10000。首次更新前不能预测。预测过程不能推进在线滤波状态；重复预测应得到相同结果。`reset()` 清空历史与诊断数据，之后必须接收新观测。

ROS 节点只通过 `std::unique_ptr<predictor::Predictor>` 调用这三个方法，并检查输出点数与有限性。`prediction` 话题继续采用带时间戳的 `nav_msgs/Path`，仅填入预测位置。该 ROS 容器必须有姿态字段，因此填单位四元数作占位，不表示预测姿态。

## 现有 IMM 与新算法

`ImmPredictor` 将现有 IMM 滤波引擎封装为统一预测器。IMM 的 15 维误差状态、模型混合和姿态处理留在内部；现有数值测试仍直接测试底层引擎。

`simulation.yaml` 中使用 `prediction_algorithm: imm`。创建入口集中在 `predictor/factory.hpp` / `src/predictor_factory.cpp`，返回值始终是 `std::unique_ptr<Predictor>`。

新增算法时：

1. 继承 `Predictor`，实现 `update`、`predict`、`reset`，内部状态自由定义。
2. 在工厂中增加名称与构造逻辑，为该算法读取或传入所需配置。
3. 将 `prediction_algorithm` 改为该名称；ROS 的轨迹输出与 Pangolin 的预测轨迹订阅无需修改。

`PoseKF` 当前继承该基类，但仍是尚未实现算法的抽象占位类，不能实例化，也未注册为可选算法。这样不会把空实现误当成可运行预测器。

## 可选诊断

仿真面板还需要当前估计位置、姿态、协方差和模型概率。这些信息属于 `core/diagnostics.hpp` 中独立的 `DiagnosticsProvider` 接口，不在 `Predictor` 的输出类型中。

IMM 适配器同时实现该接口。ROS 节点在算法提供诊断时才使用这些信息；算法仅实现 `Predictor` 也能发布预测轨迹、评估预测误差。缺失诊断在 CSV 中表示为 NaN，对应汇总指标为 JSON `null`；不伪造零协方差、零误差或模型概率。

Pangolin 的预测轨迹只依赖位置序列。真实四旋翼仍使用仿真里程计姿态；只有算法提供姿态诊断时，估计四旋翼才有相应数据。
