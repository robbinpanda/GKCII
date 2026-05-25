# 两车跟车-无人机跟随
# uav2.launch copy to PX4-Autopilot/launch
# models/ground_plane & cantilevered_light & gazebo_traffic_light copy to .gazebo/models
gnome-terminal -t "site_model_2cars" -- bash -c 'source devel/setup.bash;roslaunch site_model spawn_2cars.launch;bash'
sleep 8
gnome-terminal -t "uav2" -- bash -c 'roslaunch px4 uav2.launch'
sleep 20
gnome-terminal -t "iris0_follow" -- bash -c 'source devel/setup.bash;rosrun site_model iris0_follow.py'
sleep 70
gnome-terminal -t "traffic_light" -- bash -c 'source devel/setup.bash;rosrun site_model traffic_light.py'
gnome-terminal -t "platoon" -- bash -c 'source devel/setup.bash;rosrun site_model platoon.py "car1" "car2" "base_link2" "follow1";bash'
gnome-terminal -t "lane" -- bash -c 'source devel/setup.bash;rosrun site_model lane.py;bash'
