// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from controller:msg/MotorSpeed.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__MOTOR_SPEED__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define CONTROLLER__MSG__DETAIL__MOTOR_SPEED__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "controller/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "controller/msg/detail/motor_speed__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace controller
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_controller
cdr_serialize(
  const controller::msg::MotorSpeed & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_controller
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  controller::msg::MotorSpeed & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_controller
get_serialized_size(
  const controller::msg::MotorSpeed & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_controller
max_serialized_size_MotorSpeed(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_controller
cdr_serialize_key(
  const controller::msg::MotorSpeed & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_controller
get_serialized_size_key(
  const controller::msg::MotorSpeed & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_controller
max_serialized_size_key_MotorSpeed(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace controller

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_controller
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, controller, msg, MotorSpeed)();

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__MSG__DETAIL__MOTOR_SPEED__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
