cmake_minimum_required(VERSION 3.8)
project(controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-register -Wno-unused-variable -Wno-unused-but-set-variable -Wno-sign-compare -Wno-vla)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(maneuver REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(Eigen REQUIRED)

# Include directories for custom headers
include_directories(
  include
  ${maneuver_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS})


# Messages
set(MSG_FILES
  "msg/MotorSpeed.msg"
  "msg/Pose.msg"
  "msg/Uvector.msg"
  "msg/ErrorMsg.msg"
)

set(SRV_FILES
  "srv/AngularWrench.srv"
  "srv/LinearWrench.srv"
)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES std_msgs geometry_msgs
)

# Ensure the generated headers are available
ament_export_dependencies(rosidl_default_runtime)

# Add executables
add_executable(${PROJECT_NAME}_node  src/controller.cpp)
ament_target_dependencies(${PROJECT_NAME}_node  rclcpp std_msgs geometry_msgs maneuver)

# Correct the deprecated usage
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

target_link_libraries(${PROJECT_NAME}_node  "${cpp_typesupport_target}")

# Install targets
install(TARGETS ${PROJECT_NAME}_node  
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


# Export the package
ament_package()
