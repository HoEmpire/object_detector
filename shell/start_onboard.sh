#!/bin/zsh
source /opt/ros/melodic/setup.zsh
source /home/nvidia/project/camera_driver/devel/setup.zsh
source /home/nvidia/project/livox-ros/devel/setup.zsh
source /home/nvidia/detection_ws/devel/setup.zsh
roslaunch platform_driver platform_driver.launch & sleep 10
roslaunch spinnaker_camera_driver camera.launch
roslaunch livox_ros_driver livox_lidar_msg.launch & sleep 5
roslaunch object_detector detector.launch