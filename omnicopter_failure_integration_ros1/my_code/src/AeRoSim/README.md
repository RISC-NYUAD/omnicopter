# AeRoSim-omnicopter

Inside your ros_ws/src:

    git clone git@github.com:RISC-NYUAD/AeRoSim.git
    cd AeRoSim
    git checkout omnicopter
    cd ../..
    catkin build

The ***velocity_flight.launch*** file can start a simulation with a pre-built velocity controller, to be used with a joystick.
Check the files **attitude_controller.cpp** and **velocity_controller.cpp** to see how to use the simulator.

Update parameters such as **thrust constant**,**max RPM** etc in **models/omnicopter/model.sdf Line 691** and in the **config** folder files if you want to keep using the pre-built controllers.
