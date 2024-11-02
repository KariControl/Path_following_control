source /opt/ros/humble/setup.bash
. install/setup.bash
gnome-terminal -- ros2 launch vehicle_sim vehicle_run.py
sleep 1s
gnome-terminal -- ros2 launch path_following control_run.py
ros2 bag record steering_angle vehicle_state target_path