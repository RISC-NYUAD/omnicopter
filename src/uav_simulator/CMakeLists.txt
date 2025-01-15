cmake_minimum_required(VERSION 3.8)
project(uav_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gz-sim8 REQUIRED)  
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(maneuver REQUIRED)  # Include maneuver for FullPose.msg
find_package(Eigen3 REQUIRED)
include_directories(include
  ${EIGEN3_INCLUDE_DIRS}
  ${maneuver_INCLUDE_DIRS})

add_library(BetaMotorPlugin SHARED src/beta_motor_plugin.cpp)
target_link_libraries(BetaMotorPlugin gz-sim8::gz-sim8)
ament_target_dependencies(BetaMotorPlugin rclcpp geometry_msgs)

add_library(OdometrySensor SHARED src/odometry_plugin.cpp)
target_link_libraries(OdometrySensor gz-sim8::gz-sim8)
ament_target_dependencies(OdometrySensor rclcpp geometry_msgs nav_msgs maneuver)



install(
  TARGETS BetaMotorPlugin OdometrySensor
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
