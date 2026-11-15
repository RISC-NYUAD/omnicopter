#!/bin/bash

# Function to pause and wait for user input
pause() {
  read -p "Press [Enter] key to continue..."
}

# Function to stop the background process
stop_bg_process() {
  echo "Stopping the background command..."
  kill -SIGINT $bg_pid
  sleep 2
  if ps -p $bg_pid > /dev/null; then
    echo "Force killing the background command..."
    kill -SIGKILL $bg_pid
  fi
}

# Trap to ensure the background process is stopped on script exit
trap stop_bg_process EXIT

# Start recording
echo "Start recording"
timestamp=$(date +"%Y%m%d_%H%M%S")
ros2 bag record -a -o "rosbag_$timestamp.bag" & 
bg_pid=$!

# Arm
echo "Arm the motors?"
pause
ros2 service call /arm_disarm maneuver/srv/ArmDisarm "{}"

# Lift_off
echo "Lift off to 1.3m?"
pause
ros2 service call /lift_off maneuver/srv/LiftOff "{height: 1.3, duration: 15.0}"

# 6D trajectory
echo "Go to (1, 2.5, 1.3), (1, 0, 0)?"
pause
ros2 service call /goto_6d_point maneuver/srv/Goto6DPoint "{x: -1.0, y: -1.0, z: 1.3, roll: 1.0, pitch: 0.0, yaw: 0.0, duration: 10.0}"

# Land
echo "Initiate Landing?"
pause
ros2 service call /land maneuver/srv/Land "{height_1: 1.2, duration_1: 10.0, height_2: 0.35, duration_2: 3.0}"
pause

# Stopping the background process
ros2 node kill /my_bag
stop_bg_process

echo "All commands executed."

