# Port change record

The unmodified numerical allocator headers are copied from the adopted ROS 1 implementation. Transport and simulator integration are ported for Jazzy / Harmonic.

The original versions of modified files are retained in original_ros2_modified_files.tar.gz. This is a source backup, not a rollback command: extract into a separate folder to inspect or restore selected files. The supplied original ZIP remains the authoritative full baseline.

| File | Change |
|---|---|
| Dockerfile | modified |
| README.md | modified |
| ROS2_FAILURE_README.md | added |
| bash_Scripts/run_S1.sh | added |
| bash_Scripts/run_S2.sh | added |
| docker-compose.yml | modified |
| src/controller/CMakeLists.txt | modified |
| src/controller/config/failure_aware.yaml | added |
| src/controller/include/controller/feasible_allocator.hpp | added |
| src/controller/include/controller/governed_allocator.hpp | added |
| src/controller/msg/AllocatorStatus.msg | added |
| src/controller/package.xml | modified |
| src/controller/src/failure_aware_node.cpp | added |
| src/controller/srv/SetMotorLimits.srv | added |
| src/controller/test/governed_checks.cpp | added |
| src/controller/test/model_fixture.hpp | added |
| src/controller/tools/model_config.py | added |
| src/firmware/CMakeLists.txt | modified |
| src/firmware/launch/failure_aware.launch.py | added |
| src/firmware/package.xml | modified |
| src/firmware/scripts/run_failure_scenario.py | added |
| src/firmware/scripts/set_motor_failure.py | added |
| src/maneuver/include/maneuver/maneuver.hpp | modified |
| src/maneuver/src/maneuver.cpp | modified |
| src/uav_simulator/CMakeLists.txt | modified |
| src/uav_simulator/package.xml | modified |
| src/uav_simulator/src/beta_motor_plugin.cpp | modified |
| src/uav_simulator/src/odometry_plugin.cpp | modified |
| src/uav_simulator/src/thrust_dynamics.hpp | added |
| src/uav_simulator/test/thrust_checks.cpp | added |
| src/uav_simulator/worlds/failure.world | added |
| src/uav_simulator/worlds/models/omnicopter_failure/model.config | added |
| src/uav_simulator/worlds/models/omnicopter_failure/model.sdf | added |

See ROS2_FAILURE_README.md for behavior, commands and untested integration boundaries.
