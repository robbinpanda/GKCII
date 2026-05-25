`仿真环境部分`
# 生成带有ugv0和ugv1的仿真环境
gnome-terminal -t "gazebo_ugv" -- bash -c 'roslaunch px4 ugv0.launch'
sleep 15 # 先生成ugv再生成uav，防止uav掉到ugv下
# 生成带有uav0和uav1的仿真环境，每辆ugv上一架uav
gnome-terminal -t "gazebo_uav" -- bash -c 'roslaunch px4 uav0.launch'
sleep 45
#catkin_make

`uav部分`
# uav0和uav1键盘控制
gnome-terminal -t "iris_keyboard_control" -- bash -c 'source devel/setup.bash;rosrun control_pkg iris_control.py;bash'

`ugv部分`
gnome-terminal -t "rover0_servo_commands" -- bash -c 'source devel/setup.bash;rosrun control_pkg rover0_servo_commands.py'
# ugv0键盘控制
gnome-terminal -t "rover_keyboard_control" -- bash -c 'source devel/setup.bash;rosrun control_pkg rover0_keyboard_control.py;bash'
