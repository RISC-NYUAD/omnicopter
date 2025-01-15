# ROS2 omnicopter Collaboration
Software developed for the omnidrone simulation

The software is based on ROS2 jazzy and it uses gazebo harmonic (newer than gazebo classic and ignition).
**All units are SI including the angles (rad)**

# The software was developed in Docker environmrnt to eliminate the dependencies and Ubuntu version problems for the user.

# Building and Running
- Put all folders and files in an empty folder and navigate to this directory from terminal.
    - build docker image for first time only and run it.
        docker-compose up -d --build

    -  To run the image later remove the --build.
        docker-compose up -d --build

    - Attach to a docker session where omni is the image name.
        docker-compose exec omni bash

    - The default directory in the docker image will be the ros2 workspace and it has src and bash_Scripts folders.
    - If the workspace is not built already build it.
        colcon build
    
    - To exit from the docker terminal simply write "exit"

    - To stop the docker image
        docker-compose down

    - To purge all old docker images:
        docker system prune 

# GUI
- Since the docker image itself comes with no GUI, an Xpra server is setup to give a GUI through the web browser.
- After starting the container open the browser and go to:
    http://localhost:8080/
- In the GUI, hit applications from the list and choose xterm to have a terminal.

# Running the simulation
- From the GUI terminal, source the worksspace bash file.
    source install/setup.bash
- Run the simulation using the simulate.launch that launches all packages.
    ros2 launch firmware simulate.launch
- Hit play in the gazebo window to start.
- After this, you can use the services or the bash script files to interact with the drone.

# Services 
- ARM using arm/disarm service has to be called first. [ros2 service call /arm_disarm maneuver/srv/ArmDisarm "{}"]
- Take-off has to be the first maneuver after arming [ros2 service call /lift_off maneuver/srv/LiftOff "{height: 0.0, duration: 0.0}"]
- Land [ros2 service call /land maneuver/srv/Land "{height_1: 0.0, duration_1: 0.0, height_2: 0.0, duration_2: 0.0}"]
    Goes to these heights in the specified times to make smooth landing
- 6D trajectory to control pos and angles together [ros2 service call /goto_6d_point maneuver/srv/Goto6DPoint "{x: 0.0, y: 0.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0, duration: 0.0}"]
- Full flip on either pitch or roll or both [ros2 service call /full_flip maneuver/srv/FullFlip "{roll_bool: false, pitch_bool: true, duration: 0.0}"]
- There are other services [LinearWrench, AngularWrench, ellipse5D (**under dev**)] but not fundametal for the flights.

# Bash scripts
- Bash scripts are provided to do different manuevers each including the Arming, take-off, Maneuver, and landing while recording all the topics as Rosbags. The scripts are using the above ROS2 services.
- Access has to be given to bash script to run. [sudo chmod +x script_file].
- The script files record ros2 bags for each flight. The bags will be saved in the bash_Scripts folder.

# New features in this branch
- This branch has everything related to the simulation and **all packages and most of the codes related to the hardware implementation are removed**.
