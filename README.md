# omnicopter v1.0
Softwares developed for the omnidrone

The software is based on ROS1 and it uses MSP to communicate with the INAV on the flight controller. The code is developed for Odroid XU4.
**All units are SI including the angles (rad)**

- Upload the INAV to T-motor f7 flight controller.
- Put the src in a workspace and build.
- Simulation: roslaunch omni_firmware simulate.launch
- Real flight: roslaunch omni_firmware system.launch
- Check Raw IMU data and vicon topics.

# Services 
- ARM using Arm/Disarm service. [rosservice call /ArmDisarm]
- Take-off [rosservice call /lift_off "height: 0.0 duration: 0.0"]
- Pos + yaw Manuever always forcing 0 roll and pitch [rosservice call /go_to "{x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, duration: 0.0}"]
- Rotations (rad) while holding pos [rosservice call /rotate_to "roll: 0.0 pitch: 0.0 yaw: 0.0 duration: 0.0"]
- Land [rosservice call /land "height_1: 0.0 duration_1: 0.0 height_2: 0.0 duration_2: 0.0]
    Goes to these heights in the specified times to make smooth landing
- 6D trajectory to control pos and angles together [rosservice call /go_to_6D "{x: 0.0, y: 0.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0, duration: 0.0}"]

- There are other services [LinearWrench, AngularWrench, ellipse5D (**under dev**)] but not fundametal for the flights.

# Bash scripts
- Bash scripts are provided to do different manuevers each including the take-off and landing while recording all the topics as Rosbags. The scripts are using the above ROS services.

# Killing the drone
- Press ctrl+c in the launch file terminal to send a kill signal that drops the drone to ground on spot. Also, the INAV code is modified to send 0 thrust commands to the motors if the on board pc doesn't send thrust commands to the flight controller.

# New features in this branch
- Voltage monitoring package is added and it uses I2C through the GPIO pins of the onboard pc.
- UKF filter implemented for sensor fusiuon.
- Allocation matrix modified along with the accurate model of each motor.