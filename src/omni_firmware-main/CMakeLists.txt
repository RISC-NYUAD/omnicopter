cmake_minimum_required(VERSION 3.0.2)
project(omni_firmware)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  message_generation
  message_runtime
  std_msgs
  geometry_msgs
  cmake_modules
)
find_package(Threads REQUIRED)

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
  #INCLUDE_DIRS include
 # LIBRARIES ${PROJECT_NAME}
  #CATKIN_DEPENDS
  DEPENDS 
  Eigen
)


file(GLOB_RECURSE FC_util_SRCs FC_util/*.cpp)
#file(GLOB_RECURSE EKF_IMPL_SRCs ekf_impl/src/ekf_impl.cpp)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

include_directories(
  ${Eigen_INCLUDE_DIRS}
  )

#add_library(state SHARED src/state.cpp)
##target_link_libraries(state
#  ${GAZEBO_LIBRARIES}
#  ${catkin_LIBRARIES})

# Add the UKF library
add_library(ukf src/ukf_filter.cc src/ukf_state.cc)  # Make sure you include the source file where update is defined


add_executable(state_node src/state.cpp)
target_link_libraries(state_node ${catkin_LIBRARIES}   ${Eigen_INCLUDE_DIRS} ${Eigen_LIBRARIES} Threads::Threads) 

add_executable(filter_node src/filter.cpp)
target_link_libraries(filter_node ${catkin_LIBRARIES}   ${Eigen_INCLUDE_DIRS} ${Eigen_LIBRARIES} ukf) 

#add_dependencies(${PROJECT_NAME}_node 
#    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
#    ${catkin_EXPORTED_TARGETS}
#    kair_drone_generate_messages_cpp
#  omni_firmware_generate_messages_cpp
#	)
#install(TARGETS _node 
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
