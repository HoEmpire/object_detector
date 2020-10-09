#!/bin/zsh
source /opt/ros/melodic/setup.zsh
export ROS_MASTER_URI=http://222.31.31.101:11311
export ROS_HOST=222.31.31.51

# source /home/nvidia/detection_ws/devel/setup.zsh
# roslaunch platform_driver platform_driver.launch & sleep 12

# source /home/nvidia/project/camera_driver/devel/setup.zsh
# roslaunch spinnaker_camera_driver camera.launch  & sleep 3

# source /home/nvidia/project/livox-ros/devel/setup.zsh
# roslaunch livox_ros_driver livox_lidar_msg.launch & sleep 3

# source /home/nvidia/detection_ws/devel/setup.zsh
# roslaunch object_detector detector.launch & sleep 0.1
# wait
# exit 0