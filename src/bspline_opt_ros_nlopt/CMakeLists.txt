cmake_minimum_required(VERSION 3.5)
project(bspline_opt_ros)

add_compile_options(-std=c++17 -O2 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  std_msgs
  message_generation
  sensor_msgs
  rosconsole
  roslib
  octomap_ros
  omni_firmware
  octomap_msgs
  pcl_ros
  pcl_conversions
)

# NLopt is a system dependency; ensure it's installed (e.g., sudo apt-get install nlopt libnlopt-dev)
find_package(NLopt REQUIRED)

# --- Eigen ---
find_package(Eigen3 REQUIRED NO_MODULE)

find_package(octomap REQUIRED)
find_package(dynamicEDT3D REQUIRED)

add_service_files(
  FILES
  SetGoalPose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs nav_msgs message_runtime geometry_msgs sensor_msgs
)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${DYNAMICEDT3D_INCLUDE_DIRS}
  ${PCL_LIBRARIES}
)

add_executable(bspline_opt_node 
            src/bspline_opt_node.cpp
            src/bspline_core.cpp
            src/utils.cpp
            )
target_link_libraries(bspline_opt_node
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${DYNAMICEDT3D_LIBRARIES}
  Eigen3::Eigen
  nlopt
)
