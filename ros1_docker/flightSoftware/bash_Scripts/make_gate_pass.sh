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
rosbag record -a -O "rosbag_$timestamp.bag" /topic __name:=my_bag& 
bg_pid=$!

# Arm
echo "Arm the motors?"
pause
rosservice call /ArmDisarm

# Lift_off
echo "Lift off to 1.5m?"
pause
rosservice call /lift_off "height: 1.5
duration: 7.0" 

# Rotate
echo "Rotate to 90 degrees pitch?"
pause
rosservice call /rotate_to "roll: 0.0
pitch: -1.47
yaw: 0.0
duration: 10.0"

# 6D trajectory
echo "go to (0,1.0,1.5),(0,-1.57,0)?"
pause
rosservice call /go_to_6D "{x: 0.0, y: 1.0, z: 1.5, roll: 0.0, pitch: -1.57, yaw: 0.0, duration: 15.0}" 


# Land
echo "Initiate Landing?"
pause
rosservice call /land "height_1: 1.2
duration_1: 7.0
height_2: 0.30
duration_2: 5.0" 
pause


# Stopping the background process
rosnode kill /my_bag
stop_bg_process


echo "All commands executed."

