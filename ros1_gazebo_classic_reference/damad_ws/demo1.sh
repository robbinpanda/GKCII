`仿真环境部分`
# 生成带有ugv0和ugv1的仿真环境
gnome-terminal -t "gazebo_ugv" -- bash -c 'roslaunch px4 ugv1.launch'
sleep 15 # 先生成ugv再生成uav，防止uav掉到ugv下
# 生成带有uav0和uav1的仿真环境，每辆ugv上一架uav
gnome-terminal -t "gazebo_uav" -- bash -c 'roslaunch px4 uav1.launch'
sleep 45
#catkin_make

`红绿灯部分`
# 开启红绿灯
gnome-terminal -t "traffic_light" -- bash -c 'source devel/setup.bash;rosrun control_pkg traffic_light.py'

`uav部分`
# uav0起飞，稳定在某一高度下并识别跟随ugv0上的左边二维码
gnome-terminal -t "iris0_follow" -- bash -c 'source devel/setup.bash;rosrun control_pkg iris0_follow_qrcode.py;bash'
sleep 20
# uav1起飞，稳定在某一高度下并识别跟随ugv1上的左边二维码，iris1_callback回调函数里tag.tag_id == 16(ugv1左边二维码)
gnome-terminal -t "iris1_follow" -- bash -c 'source devel/setup.bash;rosrun control_pkg iris1_follow_qrcode.py;bash'
sleep 60
# uav0和uav1键盘控制
#gnome-terminal -t "iris_keyboard_control" -- bash -c 'source devel/setup.bash;rosrun control_pkg iris_control.py;bash'

`ugv部分`
gnome-terminal -t "rover0_servo_commands" -- bash -c 'source devel/setup.bash;rosrun control_pkg rover0_servo_commands.py'
gnome-terminal -t "rover1_servo_commands" -- bash -c 'source devel/setup.bash;rosrun control_pkg rover1_servo_commands.py'
# ugv1准备跟车
gnome-terminal -t "follow" -- bash -c 'source devel/setup.bash;rosrun control_pkg follow.py;bash'
# ugv0巡线
gnome-terminal -t "rover0_lane" -- bash -c 'source devel/setup.bash;rosrun control_pkg lane.py;bash'
# ugv0和ugv1键盘控制
#gnome-terminal -t "rover_keyboard_control" -- bash -c 'source devel/setup.bash;rosrun control_pkg rover_keyboard_control.py;bash'
# ugv0罗技G29控制
#gnome-terminal -t "g29_info" -- bash -c './g29_info.sh;bash'
#gnome-terminal -t "rover0_g29_control" -- bash -c 'source devel/setup.bash;roslaunch control_pkg rover0_g29_control.launch;bash'
