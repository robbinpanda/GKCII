# 激光雷达点云可视化
gnome-terminal -t "site_model" -- bash -c 'source devel/setup.bash;roslaunch site_model spawn.launch;bash'
sleep 20
gnome-terminal -t "pointcloud_listener" -- bash -c 'source devel/setup.bash;python3 src/site_model/src/LidCamFusion/pointcloud_listener.py;bash'
gnome-terminal -t "pointcloud_combiner" -- bash -c 'source devel/setup.bash;rosrun site_model pointcloud_combiner;bash'
gnome-terminal -t "ServoCommands" -- bash -c 'source devel/setup.bash;python3 src/site_model/src/scripts_wx/ServoCommands.py;bash'
gnome-terminal -t "KeyControl" -- bash -c 'source devel/setup.bash;python3 src/site_model/src/scripts_wx/KeyControl.py;bash'

# visualization
# 非root模式使用cpu，卡顿严重
# source devel/setup.bash
# LIBGL_ALWAYS_SOFTWARE=1 rosrun site_model visualize.py


# root模式
# sudo -s
# source devel/setup.bash
# rosrun site_model visualize.py
