cmake_minimum_required(VERSION 2.8)
project(uav_simulator)

find_package(gazebo REQUIRED)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

if("${GAZEBO_VERSION}" VERSION_LESS "8.0")
    message(FATAL_ERROR "You need at least Gazebo 8.0. Your version: ${GAZEBO_VERSION}")
else()
    message("Gazebo version: ${GAZEBO_VERSION}")
endif()

###### COMPILE RULES ######
include(CheckCXXCompilerFlag)

macro(filter_valid_compiler_flags)
  foreach(flag ${ARGN})
    CHECK_CXX_COMPILER_FLAG(${flag} R${flag})
    if(${R${flag}})
      set(VALID_CXX_FLAGS "${VALID_CXX_FLAGS} ${flag}")
    endif()
  endforeach()
endmacro()

set(CMAKE_CXX_EXTENSIONS off)  # see gazebo CMakeList

if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING
      "Choose the type of build, options are: Debug Release RelWithDebInfo Profile Check" FORCE)
endif (NOT CMAKE_BUILD_TYPE)

# Build type link flags
set (CMAKE_LINK_FLAGS_RELEASE " " CACHE INTERNAL "Link flags for release" FORCE)
set (CMAKE_LINK_FLAGS_RELWITHDEBINFO " " CACHE INTERNAL "Link flags for release with debug support" FORCE)
set (CMAKE_LINK_FLAGS_DEBUG " " CACHE INTERNAL "Link flags for debug" FORCE)
set (CMAKE_LINK_FLAGS_PROFILE " -pg" CACHE INTERNAL "Link flags for profile" FORCE)
set (CMAKE_LINK_FLAGS_COVERAGE " --coverage" CACHE INTERNAL "Link flags for static code coverage" FORCE)

set (CMAKE_C_FLAGS_RELEASE "")
if (NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" AND NOT MSVC)
  # -s doesn't work with clang or Visual Studio, see alternative in link below:
  # http://stackoverflow.com/questions/6085491/gcc-vs-clang-symbol-strippingu
  set (CMAKE_C_FLAGS_RELEASE "-s")
endif()

if (NOT MSVC)
  set (CMAKE_C_FLAGS_RELEASE " ${CMAKE_C_FLAGS_RELEASE} -O3 -DNDEBUG ${CMAKE_C_FLAGS_ALL}" CACHE INTERNAL "C Flags for release" FORCE)
  set (CMAKE_CXX_FLAGS_RELEASE ${CMAKE_C_FLAGS_RELEASE})

  set (CMAKE_C_FLAGS_RELWITHDEBINFO " -g -O2 ${CMAKE_C_FLAGS_ALL}" CACHE INTERNAL "C Flags for release with debug support" FORCE)
  set (CMAKE_CXX_FLAGS_RELWITHDEBINFO ${CMAKE_C_FLAGS_RELWITHDEBINFO})

  set (CMAKE_C_FLAGS_DEBUG " -ggdb3 ${CMAKE_C_FLAGS_ALL}" CACHE INTERNAL "C Flags for debug" FORCE)
  set (CMAKE_CXX_FLAGS_DEBUG ${CMAKE_C_FLAGS_DEBUG})

  set (CMAKE_C_FLAGS_PROFILE " -fno-omit-frame-pointer -g -pg ${CMAKE_C_FLAGS_ALL}" CACHE INTERNAL "C Flags for profile" FORCE)
  set (CMAKE_CXX_FLAGS_PROFILE ${CMAKE_C_FLAGS_PROFILE})

  set (CMAKE_C_FLAGS_COVERAGE " -g -O0 -Wformat=2 --coverage -fno-inline ${CMAKE_C_FLAGS_ALL}" CACHE INTERNAL "C Flags for static code coverage" FORCE)
  set (CMAKE_CXX_FLAGS_COVERAGE "${CMAKE_C_FLAGS_COVERAGE}")
  if (NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    # -fno-default-inline -fno-implicit-inline-templates are unimplemented, cause errors in clang
    # -fno-elide-constructors can cause seg-faults in clang 3.4 and earlier
    # http://llvm.org/bugs/show_bug.cgi?id=12208
    set (CMAKE_CXX_FLAGS_COVERAGE "${CMAKE_CXX_FLAGS_COVERAGE} -fno-default-inline -fno-implicit-inline-templates -fno-elide-constructors")
  endif()
endif()

#####################################
# Set all the global build flags
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS_${CMAKE_BUILD_TYPE_UPPERCASE}}")
set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE_UPPERCASE}}")
set (CMAKE_SHARED_LINKER_FLAGS "${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE_UPPERCASE}}")
set (CMAKE_MODULE_LINKER_FLAGS "${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE_UPPERCASE}}")

# Compiler-specific C++11 activation.
if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU")
  execute_process(
      COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
  if (NOT (GCC_VERSION VERSION_GREATER 4.7))
    message(FATAL_ERROR "${PROJECT_NAME} requires g++ 4.8 or greater.")
  endif ()
elseif ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
  execute_process(
      COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE CLANG_VERSION)
  if (NOT (CLANG_VERSION VERSION_GREATER 3.2))
    message(FATAL_ERROR "${PROJECT_NAME} requires clang 3.3 or greater.")
  endif ()

  if ("${CMAKE_SYSTEM_NAME}" MATCHES "Darwin")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
  endif ()
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  if (MSVC_VERSION LESS 1800)
    message(FATAL_ERROR "${PROJECT_NAME} requires VS 2013 or greater.")
  endif ()
else ()
  message(FATAL_ERROR "Your C++ compiler does not support C++11.")
endif ()

#set(WARN_LEVEL "-Wall")
#filter_valid_compiler_flags(${WARN_LEVEL}
#    -Wextra -Wno-long-long -Wno-unused-value -Wfloat-equal -Wshadow
#    -Wswitch-default -Wmissing-include-dirs -pedantic)
#if (UNIX AND NOT APPLE)
#  filter_valid_compiler_flags(-fvisibility=hidden -fvisibility-inlines-hidden)
#endif()

set(UNFILTERED_FLAGS "-std=c++14")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${VALID_CXX_FLAGS} ${UNFILTERED_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  geometry_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  mav_msgs
  cmake_modules
  aerosim_comm
)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${GAZEBO_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        /usr/include/eigen3
        
        )

link_libraries(
        ${GAZEBO_LIBRARIES}
        )

link_directories(
        ${GAZEBO_LIBRARY_DIRS}
        )


catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros
)


add_library(betacmd_motor_plugin SHARED src/beta_motor_plugin.cpp)
target_link_libraries(betacmd_motor_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(odometry_plugin SHARED src/odometry_plugin.cpp)
target_link_libraries(odometry_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(att_ctrl_node nodes/attitude_controller.cpp)
target_link_libraries(att_ctrl_node ${catkin_LIBRARIES} )

add_executable(adapter_node nodes/adapter.cpp)
target_link_libraries(adapter_node ${catkin_LIBRARIES} )

add_executable(vel_ctrl_node nodes/velocity_controller.cpp)
target_link_libraries(vel_ctrl_node ${catkin_LIBRARIES} )

add_executable(joy_rc_node nodes/joy_rc_sim.cpp)
target_link_libraries(joy_rc_node ${catkin_LIBRARIES} )

add_executable(joy_vel_node nodes/joy_vel_pub.cpp)
target_link_libraries(joy_vel_node ${catkin_LIBRARIES} )


