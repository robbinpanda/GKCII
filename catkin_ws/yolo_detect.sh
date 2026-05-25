# ==========================================
# yolo_detect.sh — 启动仿真环境与目标检测系统
# ==========================================

# 提示：请确保 ground_plane、cantilevered_light、gazebo_traffic_light 三个模型
# 已复制到 ~/.gazebo/models 目录，否则仿真时可能出现模型缺失。
# 示例命令：
# cp -r models/* ~/.gazebo/models/

# -------------------------------
# 启动仿真场景：包含两个小车的世界
# -------------------------------
gnome-terminal -t "site_model_2cars" -- bash -c 'source devel/setup.bash;roslaunch site_model spawn_2cars.launch;bash'

# 等待20秒，确保仿真环境与模型加载完成后再启动后续节点
sleep 20

# -------------------------------
# （可选）启动交通灯控制节点（当前已注释）
# 如果你想控制场景中的红绿灯，可以取消以下注释
# -------------------------------
# gnome-terminal -t "traffic_light" -- bash -c 'source devel/setup.bash;rosrun site_model traffic_light.py'

# -------------------------------
# 启动舵机控制脚本（控制小车方向/油门等伺服动作）
# -------------------------------
gnome-terminal -t "ServoCommands" -- bash -c 'source devel/setup.bash;python3 src/site_model/src/scripts_wx/ServoCommands.py;bash'

# -------------------------------
# 启动键盘控制节点（接收键盘输入以控制小车移动）
# 支持前进、后退、左右转等操作
# -------------------------------
gnome-terminal -t "KeyControl" -- bash -c 'source devel/setup.bash;python3 src/site_model/src/scripts_wx/KeyControl.py;bash'

# -------------------------------
# 启动YOLO目标检测节点
# 读取图像话题并进行实时检测、画框、展示
# -------------------------------
gnome-terminal -t "TagDetect_YOLO" -- bash -c 'source devel/setup.bash;rosrun site_model yolo_detect.py
  echo "Press any key to close..."
  read
'
