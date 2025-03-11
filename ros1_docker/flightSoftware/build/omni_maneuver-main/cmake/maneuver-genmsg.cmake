# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "maneuver: 0 messages, 8 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(maneuver_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/LiftOff.srv" NAME_WE)
add_custom_target(_maneuver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "maneuver" "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/LiftOff.srv" ""
)

get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/GotoPoint.srv" NAME_WE)
add_custom_target(_maneuver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "maneuver" "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/GotoPoint.srv" ""
)

get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Goto6DPoint.srv" NAME_WE)
add_custom_target(_maneuver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "maneuver" "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Goto6DPoint.srv" ""
)

get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/RotateTo.srv" NAME_WE)
add_custom_target(_maneuver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "maneuver" "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/RotateTo.srv" ""
)

get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/ArmDisarm.srv" NAME_WE)
add_custom_target(_maneuver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "maneuver" "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/ArmDisarm.srv" ""
)

get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Land.srv" NAME_WE)
add_custom_target(_maneuver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "maneuver" "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Land.srv" ""
)

get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Ellipse5D.srv" NAME_WE)
add_custom_target(_maneuver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "maneuver" "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Ellipse5D.srv" ""
)

get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/FullFlip.srv" NAME_WE)
add_custom_target(_maneuver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "maneuver" "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/FullFlip.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/LiftOff.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/maneuver
)
_generate_srv_cpp(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/GotoPoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/maneuver
)
_generate_srv_cpp(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Goto6DPoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/maneuver
)
_generate_srv_cpp(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/RotateTo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/maneuver
)
_generate_srv_cpp(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/ArmDisarm.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/maneuver
)
_generate_srv_cpp(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Land.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/maneuver
)
_generate_srv_cpp(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Ellipse5D.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/maneuver
)
_generate_srv_cpp(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/FullFlip.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/maneuver
)

### Generating Module File
_generate_module_cpp(maneuver
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/maneuver
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(maneuver_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(maneuver_generate_messages maneuver_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/LiftOff.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_cpp _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/GotoPoint.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_cpp _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Goto6DPoint.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_cpp _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/RotateTo.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_cpp _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/ArmDisarm.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_cpp _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Land.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_cpp _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Ellipse5D.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_cpp _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/FullFlip.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_cpp _maneuver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(maneuver_gencpp)
add_dependencies(maneuver_gencpp maneuver_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS maneuver_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/LiftOff.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/maneuver
)
_generate_srv_eus(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/GotoPoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/maneuver
)
_generate_srv_eus(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Goto6DPoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/maneuver
)
_generate_srv_eus(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/RotateTo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/maneuver
)
_generate_srv_eus(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/ArmDisarm.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/maneuver
)
_generate_srv_eus(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Land.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/maneuver
)
_generate_srv_eus(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Ellipse5D.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/maneuver
)
_generate_srv_eus(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/FullFlip.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/maneuver
)

### Generating Module File
_generate_module_eus(maneuver
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/maneuver
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(maneuver_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(maneuver_generate_messages maneuver_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/LiftOff.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_eus _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/GotoPoint.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_eus _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Goto6DPoint.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_eus _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/RotateTo.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_eus _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/ArmDisarm.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_eus _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Land.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_eus _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Ellipse5D.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_eus _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/FullFlip.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_eus _maneuver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(maneuver_geneus)
add_dependencies(maneuver_geneus maneuver_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS maneuver_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/LiftOff.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/maneuver
)
_generate_srv_lisp(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/GotoPoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/maneuver
)
_generate_srv_lisp(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Goto6DPoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/maneuver
)
_generate_srv_lisp(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/RotateTo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/maneuver
)
_generate_srv_lisp(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/ArmDisarm.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/maneuver
)
_generate_srv_lisp(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Land.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/maneuver
)
_generate_srv_lisp(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Ellipse5D.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/maneuver
)
_generate_srv_lisp(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/FullFlip.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/maneuver
)

### Generating Module File
_generate_module_lisp(maneuver
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/maneuver
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(maneuver_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(maneuver_generate_messages maneuver_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/LiftOff.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_lisp _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/GotoPoint.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_lisp _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Goto6DPoint.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_lisp _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/RotateTo.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_lisp _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/ArmDisarm.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_lisp _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Land.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_lisp _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Ellipse5D.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_lisp _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/FullFlip.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_lisp _maneuver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(maneuver_genlisp)
add_dependencies(maneuver_genlisp maneuver_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS maneuver_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/LiftOff.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/maneuver
)
_generate_srv_nodejs(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/GotoPoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/maneuver
)
_generate_srv_nodejs(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Goto6DPoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/maneuver
)
_generate_srv_nodejs(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/RotateTo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/maneuver
)
_generate_srv_nodejs(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/ArmDisarm.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/maneuver
)
_generate_srv_nodejs(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Land.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/maneuver
)
_generate_srv_nodejs(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Ellipse5D.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/maneuver
)
_generate_srv_nodejs(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/FullFlip.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/maneuver
)

### Generating Module File
_generate_module_nodejs(maneuver
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/maneuver
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(maneuver_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(maneuver_generate_messages maneuver_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/LiftOff.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_nodejs _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/GotoPoint.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_nodejs _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Goto6DPoint.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_nodejs _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/RotateTo.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_nodejs _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/ArmDisarm.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_nodejs _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Land.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_nodejs _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Ellipse5D.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_nodejs _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/FullFlip.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_nodejs _maneuver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(maneuver_gennodejs)
add_dependencies(maneuver_gennodejs maneuver_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS maneuver_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/LiftOff.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/maneuver
)
_generate_srv_py(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/GotoPoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/maneuver
)
_generate_srv_py(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Goto6DPoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/maneuver
)
_generate_srv_py(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/RotateTo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/maneuver
)
_generate_srv_py(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/ArmDisarm.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/maneuver
)
_generate_srv_py(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Land.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/maneuver
)
_generate_srv_py(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Ellipse5D.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/maneuver
)
_generate_srv_py(maneuver
  "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/FullFlip.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/maneuver
)

### Generating Module File
_generate_module_py(maneuver
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/maneuver
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(maneuver_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(maneuver_generate_messages maneuver_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/LiftOff.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_py _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/GotoPoint.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_py _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Goto6DPoint.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_py _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/RotateTo.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_py _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/ArmDisarm.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_py _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Land.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_py _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/Ellipse5D.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_py _maneuver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/flightSoftware/src/omni_maneuver-main/srv/FullFlip.srv" NAME_WE)
add_dependencies(maneuver_generate_messages_py _maneuver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(maneuver_genpy)
add_dependencies(maneuver_genpy maneuver_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS maneuver_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/maneuver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/maneuver
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(maneuver_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/maneuver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/maneuver
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(maneuver_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/maneuver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/maneuver
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(maneuver_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/maneuver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/maneuver
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(maneuver_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/maneuver)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/maneuver\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/maneuver
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(maneuver_generate_messages_py std_msgs_generate_messages_py)
endif()
