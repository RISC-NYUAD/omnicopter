cmake_minimum_required(VERSION 3.8)
project(controller)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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
find_path(NLOPT_INCLUDE_DIR nlopt.h REQUIRED)
find_library(NLOPT_LIBRARY nlopt REQUIRED)
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
  "msg/AllocatorStatus.msg"
)

set(SRV_FILES
  "srv/AngularWrench.srv"
  "srv/LinearWrench.srv"
  "srv/SetMotorLimits.srv"
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



add_executable(failure_aware_node src/failure_aware_node.cpp)
target_include_directories(failure_aware_node PRIVATE ${NLOPT_INCLUDE_DIR})
ament_target_dependencies(failure_aware_node rclcpp std_msgs geometry_msgs maneuver)
target_link_libraries(failure_aware_node "${cpp_typesupport_target}" ${NLOPT_LIBRARY})
install(TARGETS failure_aware_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS tools/model_config.py DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  add_executable(failure_math_checks test/governed_checks.cpp)
  target_include_directories(failure_math_checks PRIVATE include/controller test ${NLOPT_INCLUDE_DIR})
  target_link_libraries(failure_math_checks ${NLOPT_LIBRARY})
  enable_testing()
  add_test(NAME failure_math_checks COMMAND failure_math_checks)
endif()
ament_package()

