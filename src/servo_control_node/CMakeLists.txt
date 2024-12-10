cmake_minimum_required(VERSION 3.0.2)
project(servo_control_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  message_generation  # For generating services and messages
)

## Add service files
add_service_files(
  FILES
  SetAngles.srv
  ReleaseArms.srv
)

## Generate messages
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs std_srvs message_runtime
)

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Add executable
add_executable(servo_control_node src/servo_control_node.cpp)

# Link the libraries, including gpiod
target_link_libraries(servo_control_node
  ${catkin_LIBRARIES}
  ${GPIOD_LIB}  # Link to gpiod library
)

## Add dependencies to ensure messages are generated before compiling
add_dependencies(servo_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

