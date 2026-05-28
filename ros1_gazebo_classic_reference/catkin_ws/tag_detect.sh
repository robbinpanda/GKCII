# models/ground_plane & cantilevered_light & gazebo_traffic_light copy to .gazebo/models
gnome-terminal -t "site_model_2cars" -- bash -c 'source devel/setup.bash;roslaunch site_model spawn_2cars.launch;bash'
sleep 20
gnome-terminal -t "ServoCommands" -- bash -c 'source devel/setup.bash;python3 src/site_model/src/scripts_wx/ServoCommands.py;bash'
gnome-terminal -t "KeyControl" -- bash -c 'source devel/setup.bash;python3 src/site_model/src/scripts_wx/KeyControl.py;bash'
gnome-terminal -t "TagDetect" -- bash -c 'source devel/setup.bash;rosrun site_model tag_detect.py;bash'
