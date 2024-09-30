cmake_minimum_required(VERSION 2.8)
project(aerosim_comm)


find_package(catkin REQUIRED COMPONENTS
  message_generation
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        )

add_service_files(
  FILES
  ModeUpdate.srv
)

add_message_files(
  FILES
  Training.msg
  ArmAction.msg
)

generate_messages(DEPENDENCIES )

catkin_package(
  DEPENDS 
    message_runtime
)

