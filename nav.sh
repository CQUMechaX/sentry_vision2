#! /bin/bash

colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON 
cmds=(  "ros2 launch rm_bringup bringup.launch.py"
	"ros2 launch livox_ros_driver2 msg_MID360_launch.py"
	"ros2 launch linefit_ground_segmentation_ros segmentation.launch.py" 
	"ros2 launch fast_lio mapping.launch.py"
	"ros2 launch imu_complementary_filter complementary_filter.launch.py"
	"ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py"
	"ros2 launch icp_localization_ros2 bringup.launch.py"
	"ros2 launch rm_navigation bringup_launch.py "
	#"ros2 launch rm_decision my_launch.py"
	#"ros2 launch rm_serial_driver serial_driver.launch.py"
	"ros2 launch rm_vision_bringup vision_bringup.launch.py"
	)

for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -ic "source ~/.bashrc;cd $(pwd);source install/setup.bash;$cmd;"
	sleep 0.2
done
