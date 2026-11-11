# 差分 Transformer-LSTM 与 LIMM 复现

本目录是可独立安装、可直接拆分到新仓库的 Python 包，不依赖 ROS/C++。
实现 Tang 等人在 TR-C 174 (2025) 105115 中提出的三路差分预测与学习式 IMM。
[原论文](../assets/paper/1-s2.0-S0968090X25001196-main.pdf)；
[实现计划与公式映射](../assets/plan/20261111-limm.md)；
[已执行实验报告](../assets/reports/20261111-limm-experiment.md)。

已完成算法、两阶段训练、评估、权重重载、仿真 CSV 回放和 ROS 在线位置输入适配。
仓库没有作者的 MavicAir2/Payload 等17组原始数据或权重，**尚未复现论文表2/3的数值结果**。
随附合成数据生成器仅用于检验实现，不作为论文性能证据。

## 安装

在工作区根目录执行，使用独立环境避免影响 ROS 的系统 NumPy：

```bash
uv venv .venv-limm --python /usr/bin/python3
uv pip install --python .venv-limm/bin/python -r learning/requirements-cpu.txt
uv pip install --python .venv-limm/bin/python --no-build-isolation --no-deps -e learning
.venv-limm/bin/uav-limm --help
.venv-limm/bin/python -m pytest learning/tests -q
```

没有 uv 时可用 `python3 -m venv` 和环境内的 pip。CPU 锁定文件记录本次验证版本；
GPU 环境请先按 [PyTorch 官方安装说明](https://pytorch.org/get-started/locally/) 安装适合驱动的版本，
再 `pip install -e 'learning[test]'`。代码只调用官方 Transformer/LSTM/LSTMCell 等基础算子，
没有复制其他仓库的模型实现。PyTorch 是外部依赖，其许可证由该项目维护。

## 快速运行已验证的小实验

以下输出目录必须不存在，程序拒绝覆盖已有实验。数据包含6条训练、2条验证、3条测试飞行；
测试飞行使用不同频率范围，随机种子42。每条120个1 Hz点，历史40点、预测20点。

```bash
.venv-limm/bin/uav-limm generate --output outputs/limm_data --samples 120
.venv-limm/bin/uav-limm train \
  --data outputs/limm_data --output outputs/limm_run \
  --config learning/configs/paper.json --hidden 32 --feedforward 128 \
  --upstream-epochs 5 --fusion-epochs 8 --batch-size 64
.venv-limm/bin/uav-limm evaluate \
  --checkpoint outputs/limm_run/model.pt --data outputs/limm_data/test \
  --output outputs/limm_run/evaluation.json
.venv-limm/bin/uav-limm predict \
  --checkpoint outputs/limm_run/model.pt --data outputs/limm_data/test \
  --output outputs/limm_run/predictions.csv
PYTHONNOUSERSITE=1 /usr/bin/python3 scripts/plot_limm.py \
  --run outputs/limm_run --output outputs/limm_run/diagnostics.png
```

输出 `model.pt` 包含架构、完整权重、训练集拟合的尺度和来源 SHA256。
`upstream_best.pt` / `fusion_best.pt` 是各阶段验证集最佳 state_dict；CLI 推理使用 `model.pt`。
`training.json` 保存配置、版本、种子、样本数、训练和验证损失；`metrics.json` 保存
P/V/A/LIMM/CV/CA 在相同测试窗口上的 ADE、FDE、逐步DE与终点三轴MAE/RMSE（米）。
`predictions.csv` 保存飞行标识、预测起点/目标时间、世界坐标预测/真值和欧氏误差。
目前命令行不支持中断后恢复优化器状态；`model.pt` 用于恢复推理。

`--backbone transformer_lstm` 是论文改进结构；`transformer` 去掉输出LSTM；
`lstm` 使用论文的 LSTM_F 基线。其他参数可通过 JSON 或 CLI 修改，CLI 优先。
不要将小实验的5/8轮误解为作者训练轮数：论文未报告该值。

## 自己的数据与现有仿真

每个 CSV 只能包含一条连续飞行，单位是米和秒，共用右手世界坐标系。
支持以下两种列名：

```text
t,px,py,pz
t,measurement_x,measurement_y,measurement_z,truth_x,truth_y,truth_z
```

第二种直接兼容现有仿真的 `samples.csv`，输入只取 measurement，truth 只作训练/评估标签。
第一种从位置序列自身构造未来标签。GPS 经纬度需先转换到当地 Cartesian 坐标，不能把度当米输入。
API `resample_csv(columns=..., target_columns=...)` 支持其他列名。
数据按 `dt` 线性重采样，每条飞行的首个观测点作为坐标原点；默认拒绝超过 `2*dt` 的缺测区间，
请先分段。离线插值可能用到采样时刻右侧的已记录点，属于离线数据预处理；在线节点不做该插值。

训练数据可组织为 `data/train/*.csv`、`data/val/*.csv`、`data/test/*.csv`，三者必须为独立飞行，
推荐按整条飞行时间顺序分配7:1:2，符合论文流程。也可以 `--data flight1.csv flight2.csv`，
程序会在每条飞行内部按时间切分7:1:2，随后生成窗口，避免重叠窗口跨划分泄漏。
这两种划分方法不同，正式对比论文时应使用前者，并记录飞行划分。
所有训练尺度仅从训练部分观测拟合，验证和测试只复用这些尺度及各自历史范围。

注意 `train` 自动/显式划分，而 `evaluate` / `predict` 使用传入文件的**全部**窗口，
请只传测试数据。评估程序不会自动排除训练文件。
默认40点历史+20点未来，在1 Hz下每个划分/飞行至少需要60点。现有默认30秒仿真太短，
请延长记录或重新训练更短历史/不同dt的模型。改变时间步会改变训练任务，不能只在推理时改参数。

## ROS2 在线预测

C++ `predictor_node` 新发布 `/measurement_position`（Odometry消息，仅填入含噪位置），
与其内部滤波使用同一观测；LIMM 仅读取时间戳与位置。
重新编译 predictor 包，在启动数值仿真之前另开终端运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
.venv-limm/bin/python -m uav_limm.ros_node --ros-args \
  -p checkpoint:=/absolute/path/to/model.pt \
  -p observation_topic:=/measurement_position \
  -p prediction_topic:=/limm/prediction
```

仿真时长应超过历史预热时间，推荐90秒；现有 C++ 预测器同步运行以提供观测。
节点按权重的dt采样，每个格点取第一条到达的观测；允许5%采样抖动，缺测后重新预热，
不读取未来点、不借助真值补齐、不在缺少40点历史时伪造预测。
输出 `nav_msgs/Path` 包含未来20点，时间为最后输入时间加 `dt..20*dt`，与输入共用 frame。

这是独立预测话题，**没有将 `prediction_algorithm: limm` 加入 C++ 工厂**。
原有 Pangolin 和统计文件仍对应 C++ 预测器；LIMM 请使用 RViz查看 `/limm/prediction`，
或用上述离线 `evaluate` / `predict` 获取其数值结果。
ROS 不作为此包的 pip 依赖，需从系统安装并 source；测试DDS实际通信：

```bash
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=92 ROS_LOCALHOST_ONLY=1 .venv-limm/bin/python \
  learning/tests/ros_smoke.py --checkpoint outputs/limm_run/model.pt
```

无ROS时也可通过 `StreamingPredictor` 使用 `update(timestamp, xyz)` / `predict(step, count)` /
`reset()`；输入须已按模型dt采样，输出严格为未来XYZ，时间间隔错误、历史不足均明确报错。

## 算法细节和复现假设

| 论文部分 | 本实现 |
| --- | --- |
| §3.2 / Eq.(1) | 历史P/V/A长度40/39/38；独立预测20步；V累积积分、A二阶离散积分 |
| Fig.3 / §5.1 | 3层Transformer编码/解码，历史末20点+20零占位，因果mask；解码输出接LSTM再线性投影，整段预测 |
| Eq.(2)-(10) | 完整IMM交互、滤波、Gaussian似然、模式更新、均值/协方差组合，未省略交互 |
| Fig.4 / Eq.(11)-(12) | 2层历史LSTM；每个模式各自fx/fQ/fR LSTMCell，以末端编码h/c初始化；P预测省略F/Jacobian |
| §4.2(c) | logits[Tpred,3,3]逐行softmax，跨样本共享、随预测步变化 |
| Eq.(13)-(15) | 训练MaxAbs+1.5IQR；只有历史超界的轴才按该历史范围重缩放 |
| §5.1(c) | AdamW、lr .001、weight_decay .001、默认batch128，每6000优化步lr乘.9 |
| §5.3 | 融合三路转换后的未来速度，归一化速度MSE训练，最终积分回物理位置 |

以下细节论文没有完整说明，属于本实现的明确选择：隐藏64、4头、FFN256、dropout .1，
正弦位置编码、因果decoder mask、输出LSTM1层、LIMM历史编码输入为按时间对齐的归一化P/V/A共9维，
各模式统一3维速度状态、初始均匀模式概率、初始协方差可训练、上游预训练后冻结、梯度裁剪10、
验证集最佳权重和默认patience10。训练/验证归一化都仅根据历史决定动态范围。
LIMM状态统一使用历史速度范围缩放，避免直接混合不同量纲的P/V/A。
这里没有把论文的时序共享参数替换成额外条件化转移网络。

Q/R头输出6维Cholesky因子，softplus对角与小正则保证正定；通过Cholesky solve求增益和
log Gaussian似然，使用Joseph形式更新P，避免显式逆和直接概率下溢。这些数值稳定性处理
与论文简单协方差更新在精确算术下等价，但不是作者公布的实现。
浮点舍入、不同PyTorch内核/设备可能产生微小差异；种子和CPU单线程设置便于重复实验。
官方接口参考：[Transformer](https://docs.pytorch.org/docs/stable/generated/torch.nn.Transformer.html)、
[LSTMCell](https://docs.pytorch.org/docs/stable/generated/torch.nn.LSTMCell.html)、
[cholesky_solve](https://docs.pytorch.org/docs/stable/generated/torch.cholesky_solve.html)。

本目录的 `backbones.py` 不依赖数据、归一化或LIMM，可单独复用；整个 `learning/` 复制后
可独立 `pip install -e .`。暂不建立额外远端仓库，避免主项目依赖未发布的子模块。
