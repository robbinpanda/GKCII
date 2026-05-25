# 毫米波雷达-相机融合实验

## 程序关闭后，结果保存在/catkin_ws/src/site_model/output/RadCamFusion/scene中

## 如果重新编译找不到msg包或者出现没有毫米波雷达topic等问题，可以尝试使用如下命令单独编译各个功能包
## catkin_make -DCATKIN_WHITELIST_PACKAGES="package_name",毫米波雷达应该编译ARS_gazebo_plugin中的radar_plugin

gnome-terminal -t "site_model" -- bash -c 'source devel/setup.bash;roslaunch site_model spawn.launch;bash'
sleep 20
gnome-terminal -t "radar_listener" -- bash -c 'source devel/setup.bash;rosrun site_model radar_listener.py;bash'
gnome-terminal -t "ServoCommands" -- bash -c 'source devel/setup.bash;python3 src/site_model/src/scripts_wx/ServoCommands.py;bash'
gnome-terminal -t "KeyControl" -- bash -c 'source devel/setup.bash;python3 src/site_model/src/scripts_wx/KeyControl.py;bash'
gnome-terminal -t "fusion" -- bash -c 'source devel/setup.bash;cd src/site_model;python3 -m src.RadCamFusion.fusion --save;bash'
