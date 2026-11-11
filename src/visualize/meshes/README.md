# 四旋翼模型

- `hummingbird.mesh`：从本仓库历史提交 `90fdaaa` 的 `src/ego-planner-swarm/src/uav_simulator/Utils/odom_visualization/meshes/hummingbird.mesh` 恢复的原始 Ogre 模型，字节内容保持不变。
- `hummingbird.ply`：由原模型离线转换得到的三角网格，供 Pangolin 读取。保持原有顶点坐标、朝向和单位，不依赖旧显示节点。仅保留几何与法线，颜色随 `visualization.yaml` 中的 `theme` 统一切换。

模型原始包围盒约为 `0.533 × 0.533 × 0.073 m`。Pangolin 默认显示倍率为 2，机体系采用里程计原有约定。额外的机头箭头沿 +X；机体系坐标轴 X 红、Y 绿、Z 蓝。

运行时只需要已安装的 Pangolin，不需要 Ogre/Assimp。若要重新转换，系统需具备 Ogre 1.x 与 Assimp 开发库，在工作区根目录运行：

```bash
g++ -std=c++14 scripts/tools/convert_drone_mesh.cpp \
  $(pkg-config --cflags --libs OGRE) -lassimp -o /tmp/convert_drone_mesh
/tmp/convert_drone_mesh \
  src/visualize/meshes/hummingbird.mesh \
  /tmp/hummingbird-v18.mesh \
  src/visualize/meshes/hummingbird.ply
```

中间文件将原始 Ogre v1.41 升级为 v1.8，以兼容 Assimp 的导入器；随后转换为二进制 PLY。原始 `.mesh` 保留供追溯。
