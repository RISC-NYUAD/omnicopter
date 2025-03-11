# Install script for directory: /opt/ros/noetic/flightSoftware/src/omni_maneuver-main

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/opt/ros/noetic/flightSoftware/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/maneuver/srv" TYPE FILE FILES
    "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/LiftOff.srv"
    "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/GotoPoint.srv"
    "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Goto6DPoint.srv"
    "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/RotateTo.srv"
    "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/ArmDisarm.srv"
    "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Land.srv"
    "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Ellipse5D.srv"
    "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/FullFlip.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/maneuver/cmake" TYPE FILE FILES "/opt/ros/noetic/flightSoftware/build/omni_maneuver-main/catkin_generated/installspace/maneuver-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/opt/ros/noetic/flightSoftware/devel/include/maneuver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/opt/ros/noetic/flightSoftware/devel/share/roseus/ros/maneuver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/opt/ros/noetic/flightSoftware/devel/share/common-lisp/ros/maneuver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/opt/ros/noetic/flightSoftware/devel/share/gennodejs/ros/maneuver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/opt/ros/noetic/flightSoftware/devel/lib/python3/dist-packages/maneuver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/opt/ros/noetic/flightSoftware/devel/lib/python3/dist-packages/maneuver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/opt/ros/noetic/flightSoftware/build/omni_maneuver-main/catkin_generated/installspace/maneuver.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/maneuver/cmake" TYPE FILE FILES "/opt/ros/noetic/flightSoftware/build/omni_maneuver-main/catkin_generated/installspace/maneuver-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/maneuver/cmake" TYPE FILE FILES
    "/opt/ros/noetic/flightSoftware/build/omni_maneuver-main/catkin_generated/installspace/maneuverConfig.cmake"
    "/opt/ros/noetic/flightSoftware/build/omni_maneuver-main/catkin_generated/installspace/maneuverConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/maneuver" TYPE FILE FILES "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/package.xml")
endif()

