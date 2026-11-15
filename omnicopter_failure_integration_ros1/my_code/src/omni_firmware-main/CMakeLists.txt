cmake_minimum_required(VERSION 3.0.2)
project(omni_firmware)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  message_runtime
  std_msgs
  geometry_msgs
  cmake_modules
)

find_package(Eigen3 REQUIRED)
find_package(Eigen REQUIRED)

add_message_files(
   FILES
   errorMsg.msg
   FullPose.msg
   MotorSpeed.msg
   Pose.msg
   Uvector.msg
)

add_service_files(
   FILES
   ZeroUKF.srv
)
 
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   geometry_msgs
 )
 
catkin_package(
  DEPENDS 
  Eigen
)


file(GLOB_RECURSE FC_util_SRCs FC_util/*.cpp)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

include_directories(
  ${Eigen_INCLUDE_DIRS}
  )

# Add the UKF library
add_library(ukf src/ukf_filter.cc src/ukf_state.cc)  # Make sure you include the source file where update is defined


add_executable(filter_node src/filter.cpp)
target_link_libraries(filter_node ${catkin_LIBRARIES}   ${Eigen_INCLUDE_DIRS} ${Eigen_LIBRARIES} ukf) 

