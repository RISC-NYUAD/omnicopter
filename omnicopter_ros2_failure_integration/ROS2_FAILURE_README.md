# Failure-aware OmniOcta: ROS 2 Jazzy / Gazebo Harmonic

This is a separate working copy of the ROS 2 workspace you supplied. The original ZIP and your ROS 1 workspace are not modified. The adopted two-stage allocator is reused without changing its ROS-independent numerical headers.

## First build (native Ubuntu 24.04 / existing Jazzy + Harmonic installation)

Open a fresh terminal without an active ROS 1 environment. From this workspace:

```bash
sudo apt update
sudo apt install libnlopt-dev libeigen3-dev python3-numpy python3-yaml ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-default-plugins
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The original simulation's other dependencies (Gazebo Harmonic development packages, libkdtp, ROS messages) must already be available. If using rosdep, install the repository's dependencies in your usual way. Do not source the ROS 1 workspace in this terminal.

## Run the failure-aware simulation manually

```bash
ros2 launch firmware failure_aware.launch.py
```

The failure launch starts the simulation unpaused, the failure controller, and the maneuver node. Do not also run the old simulate launch or a second controller.

Arm and take off:

```bash
ros2 service call /arm_disarm maneuver/srv/ArmDisarm "{}"
ros2 service call /lift_off maneuver/srv/LiftOff "{height: 1.5, duration: 7.0}"
```

The motor-bound service is:

```text
/failure_controller/set_motor_limits  controller/srv/SetMotorLimits
```

Convenient one-motor injection / restoration:

```bash
ros2 run firmware set_motor_failure.py 4 0
ros2 run firmware set_motor_failure.py 5 0.333
ros2 run firmware set_motor_failure.py 4 1
```

These commands preserve other motors' current limits. They change the plant target limit and inform the controller immediately; actual thrust follows the motor dynamics.

Alternatively supply eight lower and eight upper bounds in newtons. User motor 1 is array index 0 / rotor_0. Nominal upper bounds are 38.63365518267137 N for each motor. A complete failure sets both bounds for that motor to zero; a partial failure sets the upper bound to the remaining fraction of the nominal upper bound.

## Automated S1 / S2, no video

Close any running simulation. From the workspace root:

```bash
bash bash_Scripts/run_S1.sh
# Run separately, after S1 exits:
bash bash_Scripts/run_S2.sh
```

Alternatively pass the workspace path as the first argument, including when the scripts are elsewhere. One Enter starts the flight after recording is ready.

- S1: complete motor 4 failure (rotor_3).
- S2: complete motor 1 failure (rotor_0).
- Arm, take off to 1.5 m over 7 s, hover 10 s, inject failure, observe 30 s.
- The failure stays active through the same two-stage landing command and disarming.
- The existing landing service ends at 0.35 m. This shutdown sequence does not establish the safe-touchdown result required by S6.
- The script checks low height and low speed before toggling disarm; on failure it saves diagnostics and stops only the launch it started.
- Results go to workspace/simulation_results. Override with OMNI_RESULTS_DIR.
- No video recording or screen-capture dependencies.

Each run produces a rosbag2 directory (metadata.yaml + SQLite .db3), controller_parameters.json, scenario.json, recording_qos.yaml, events.jsonl, logs, and COMPLETE or INTERRUPTED.txt. Upload the **whole run directory zipped**, including metadata.yaml, for analysis. ROS 1 .bag files and ROS 2 bags are different formats.

## Recorded signals

/pose_full, /pose_d, /failure_controller/status,
/omnicopter/motor_thrust, /omnicopter/motor_thrust_actual,
/omnicopter/motor_limits, /clock, /toggle_topic,
/rosout, /parameter_events, /simulation_scenarios/events.

Gazebo model_states is a Classic topic and is not available in this Harmonic launch. /pose_full supplies simulator truth at the aggregate center of mass.

Motor limits use reliable transient-local QoS, so the recorder and plugin receive the initial limits and later updates. The recorder explicitly uses matching limit and clock QoS. Events include simulation time and wall time; failure request/confirmation are also visible in the recorded motor-limit change.

## What changed

- Added a native rclcpp failure controller and ROS 2 AllocatorStatus / SetMotorLimits interfaces.
- The same adopted static-attitude selection + bounded current-wrench allocation. Reserve 1.5 N, gamma 0.1, 250 Hz. No added rate governor or actuator prediction.
- Preserved the original nominal controller and its launch. The new failure launch does not start it.
- Motor plugin accepts either direct thrust for the failure model or legacy normalized prop_cmd for the original model.
- Replaced callback-count motor smoothing with an exact first-order thrust update based on Gazebo dt. tau_up=0.15 s; no-braking tau_down=0.21 s. A failed motor's actual thrust decays; it is not clipped to zero.
- In direct mode, commands are inhibited until bounds arrive; invalid commands are ignored; stale commands drive targets to zero after 0.2 simulated seconds.
- Added one authoritative /clock publisher stamped from Gazebo simulation time.
- State publication uses world-frame COM position / linear velocity, body-frame angular velocity, and kinematic acceleration. The old extra rotation of the linear-velocity component and wall-clock stamps are removed.
- Maneuver sampling and publication now share a 10 ms simulation-time period. Previously the uploaded ROS 2 code sampled with period=100 ms but played at 100 Hz.
- Added an installed, offline failure world and separate failure model; no downloaded ground-plane/sun models or /ros_ws paths are required by the new launch.
- Model mass/inertia/allocation are regenerated from the supplied ROS 2 SDF. Aggregate mass is 3.44 kg; the physical parameters agree with the prior ROS 1 integration.
- New diagnostics include explicit desired moment and force/moment residual vectors (command-allocation errors, not measured physical wrench errors).

Generate model settings again after changing geometry:

```bash
python3 src/controller/tools/model_config.py
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Geometry changes must update controller settings and the failure model together. The generator assumes the supplied eight-rotor SDF structure and the legacy 1900-us thrust ceiling used for the earlier tests.

## Docker

The original Dockerfile is updated to install NLopt / bag dependencies before building. Hardware-only device mappings were removed from docker-compose.yml. Results are mounted persistently:

```bash
docker compose up -d --build
docker compose exec omni bash
# inside the container / browser Xpra desktop terminal:
source /ros_ws/install/setup.bash
bash /ros_ws/bash_Scripts/run_S1.sh /ros_ws
```

Use the original Xpra instructions to view Gazebo. No X11 recording is needed.

## Validation and limits

The ROS-independent allocator is compiled and tested locally, including healthy hover, S1/S2 target feasibility, hard limits, failed motor zero command, restoration, infeasible cases and numerical parity with the copied core. Actuator acceleration / zero-braking decay / residual thrust and timestep invariance are tested. Python/shell/XML/SDF are checked.

**This environment has no ROS 2 Jazzy or Gazebo Harmonic runtime. A full colcon build, DDS integration and actual Gazebo flight have not been executed here. Start with S1 and inspect its logs and bag before accepting S2 results or reproducing paper claims.** Numerical parity does not guarantee identical trajectories across Gazebo engines.

In a Jazzy environment, run:

```bash
colcon test --packages-select controller uav_simulator
colcon test-result --verbose
```

No detection algorithm is added: the controller and plant receive the same bound change without an intentional detection delay. S3-S6 remain the planned scenarios; only the previously implemented S1/S2 automation is ported here.
