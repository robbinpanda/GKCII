# ROS1 + Gazebo Classic 旧参考资料

这个目录保存 Ubuntu 20.04 / ROS Noetic / Gazebo Classic 路线的旧代码和资料，仅作为参考。

## 内容

| 路径 | 说明 |
|---|---|
| `catkin_ws/` | 旧 ROS1 主工作空间，包含 Gazebo 场景、车辆模型、传感器融合、UAV/UGV 控制脚本 |
| `damad_ws/` | 旧 ROS1 多车/多无人机演示工作空间 |
| `gazebo_home_snapshot/` | 从云端下载的 `.gazebo` 快照，主要用于查找旧 Gazebo Classic 模型 |
| `legacy_docs/` | 旧环境配置与旧 ROS1 复现方案 |
| `extra_papers/` | 早期收集的论文资料 |

## 注意

这些文件不建议在 Ubuntu 24.04 上原样运行。若必须运行旧工程，建议使用 Ubuntu 20.04 + ROS Noetic + Gazebo Classic 的 Docker 或虚拟机。

构建产物 `build/`、`devel/`、大模型权重、Gazebo 缓存和 Windows `Zone.Identifier` 文件已通过根目录 `.gitignore` 排除。

