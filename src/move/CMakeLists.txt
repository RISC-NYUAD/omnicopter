cmake_minimum_required(VERSION 3.1.3)
project(move)

## Find required packages
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  interactive_markers
  roscpp
)

## Declare catkin package
catkin_package()

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Add executables
add_executable(collision_check src/collision_check.cpp)
target_link_libraries(collision_check ${catkin_LIBRARIES})

## Install launch and config files
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Install the executables
install(TARGETS collision_check
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
