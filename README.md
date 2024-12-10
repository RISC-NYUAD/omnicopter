# omnicopter Collaboration
Software developed for the omnidrone simulation

The software is based on ROS1 and it uses mav_comm in the gazebo simulation only.
ROS1 noetic and gazebo version 11.11.0 were used for development.
**All units are SI including the angles (rad)**

# Needed External Dependencies
- Eigen
- Eigen3

# Building
- Put the src in a workspace and build (catkin_make).
   -**catkin_make has to be run several times for successful building.**
- Simulation: roslaunch omni_firmware simulate.launch

# Services 
- ARM using Arm/Disarm service has to be called first. [rosservice call /ArmDisarm]
- Take-off has to be the first maneuver after arming [rosservice call /lift_off "height: 0.0 duration: 0.0"]
- Pos + yaw Maneuver always forcing 0 roll and pitch [rosservice call /go_to "{x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, duration: 0.0}"]
- Rotations (rad) while holding pos [rosservice call /rotate_to "roll: 0.0 pitch: 0.0 yaw: 0.0 duration: 0.0"]
- Land [rosservice call /land "height_1: 0.0 duration_1: 0.0 height_2: 0.0 duration_2: 0.0]
    Goes to these heights in the specified times to make smooth landing
- 6D trajectory to control pos and angles together [rosservice call /go_to_6D "{x: 0.0, y: 0.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0, duration: 0.0}"]

- There are other services [LinearWrench, AngularWrench, ellipse5D (**under dev**)] but not fundametal for the flights.

# Bash scripts
- Bash scripts are provided to do different manuevers each including the Arming, take-off, Maneuver, and landing while recording all the topics as Rosbags. The scripts are using the above ROS services.
- Access has to be given to bash script to run. [sudo chmod +x script_file]

# New features in this branch
- This branch has everything related to the simulation and **all packages and most of the codes related to the hardware implementation are removed**.
