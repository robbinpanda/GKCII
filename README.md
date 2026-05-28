# GKCII

本仓库已按复现实验路线整理为两个主目录：

## 目录结构

| 目录 | 用途 |
|---|---|
| `ros2_px4_gazebo_ubuntu24_reproduction/` | 当前主线：Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic + PX4 的复现实验方案与目标论文 |
| `ros1_gazebo_classic_reference/` | 旧参考：ROS1/catkin/Gazebo Classic 代码、脚本、模型快照和旧环境文档 |

## 当前建议路线

优先阅读：

- `ros2_px4_gazebo_ubuntu24_reproduction/docs/px4_ros2_gazebo_ubuntu24_plan.md`
- `ros2_px4_gazebo_ubuntu24_reproduction/target_paper/Interactive_Real-time_Leader_Follower_Control_System_for_UAV_and_UGV.pdf`

旧 `catkin_ws` / `damad_ws` 不建议在 Ubuntu 24.04 上原样运行，只作为控制逻辑参考。

## GitHub 上传说明

`.gitignore` 已排除 ROS 构建产物、Gazebo 日志/缓存、Windows 下载标记、大模型权重、`node_modules` 和实验输出。旧参考目录中的源码与文档可上传，机器生成物不上传。

