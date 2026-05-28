# ROS 2 + PX4 + Gazebo / Ubuntu 24.04 主复现目录

这是当前推荐使用的复现实验路线。

## 内容

| 路径 | 说明 |
|---|---|
| `docs/px4_ros2_gazebo_ubuntu24_plan.md` | Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic + PX4 环境配置与实验路线 |
| `docs/px4_ros2_gazebo_ubuntu24_plan.pdf` | 上述方案的 PDF 版本 |
| `target_paper/` | 复现目标论文 |
| `tools/pdf_header.tex` | Markdown 转 PDF 时使用的 LaTeX 头文件 |

## 推荐实施顺序

1. 安装 ROS 2 Jazzy。
2. 安装 Gazebo Harmonic。
3. 安装 PX4-Autopilot。
4. 跑通 `make px4_sitl gz_x500`。
5. 跑通 Micro XRCE-DDS Agent 与 ROS 2 话题。
6. 新建 ROS 2 包实现论文算法：UAV leader、Mission Server、UGV follower、栅格绕障。

