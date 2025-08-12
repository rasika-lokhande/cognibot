#!/bin/bash
echo "Launching ROS2 unity..."
ros2 launch cognibot_bringup cognibot_unity.launch.xml
# ros2 run common_coding lidar_processor --ros-args -p use_sim_time:=true
# ros2 run common_coding action_executor --ros-args -p use_sim_time:=true