# 双无人机巡航
# uav2.launch copy to PX4-Autopilot/launch

gnome-terminal -t "site_model_2cars" -- bash -c 'source devel/setup.bash;roslaunch site_model spawn.launch;bash'
sleep 8
gnome-terminal -t "uav2" -- bash -c 'roslaunch px4 uav2.launch'
sleep 20
gnome-terminal -t "iris0_cruise" -- bash -c 'source devel/setup.bash;rosrun site_model iris0_cruise.py;bash'
gnome-terminal -t "iris1_cruise" -- bash -c 'source devel/setup.bash;rosrun site_model iris1_cruise.py;bash'
