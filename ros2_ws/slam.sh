#!/bin/bash

echo "[INFO] Launching robot sim"
ros2 launch cognibot_bringup cognibot_unity.launch.xml

echo "[INFO] Starting slam_toolbox node manually..."
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  -p use_sim_time:=true \
  -p publish_tf:=true \
  -p provide_odom_frame:=true \
  -p map_frame:=map \
  -p odom_frame:=odom \
  -p base_frame:=base_link \
  -p scan_topic:=/scan &
SLAM_PID=$!

# Wait a few seconds for node to start and register
sleep 2

echo "[INFO] Configuring slam_toolbox lifecycle node..."
ros2 lifecycle set /slam_toolbox configure

sleep 1

echo "[INFO] Activating slam_toolbox lifecycle node..."
ros2 lifecycle set /slam_toolbox activate

# Trap Ctrl+C to kill background process cleanly
trap "echo -e '\n[INFO] Shutting down...'; kill $SLAM_PID; exit" SIGINT

# Keep script running until user stops it
wait
