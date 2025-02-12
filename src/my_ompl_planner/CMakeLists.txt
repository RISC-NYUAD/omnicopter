cmake_minimum_required(VERSION 3.1.3)
project(my_ompl_planner)

## Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  interactive_markers
  omni_firmware  # Dependency for FullPose message
)

find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    interactive_markers
    omni_firmware
)

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Add executables
add_executable(ompl_planner_node src/ompl_planner_node.cpp)

add_definitions(${PCL_DEFINITIONS})

## Link dependencies
target_link_libraries(ompl_planner_node
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${PCL_LIBRARIES}
)


## Install launch and config files
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "*.launch" EXCLUDE)

## Install the executables
install(TARGETS ompl_planner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install Python scripts
catkin_install_python(PROGRAMS
  src/plan_traj.py  # Path to your Python script
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
