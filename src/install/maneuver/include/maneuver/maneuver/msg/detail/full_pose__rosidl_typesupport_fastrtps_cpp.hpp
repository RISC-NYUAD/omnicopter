// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from maneuver:msg/FullPose.idl
// generated code does not contain a copyright notice

#ifndef MANEUVER__MSG__DETAIL__FULL_POSE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define MANEUVER__MSG__DETAIL__FULL_POSE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "maneuver/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "maneuver/msg/detail/full_pose__struct.hpp"

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

namespace maneuver
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_maneuver
cdr_serialize(
  const maneuver::msg::FullPose & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_maneuver
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  maneuver::msg::FullPose & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_maneuver
get_serialized_size(
  const maneuver::msg::FullPose & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_maneuver
max_serialized_size_FullPose(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_maneuver
cdr_serialize_key(
  const maneuver::msg::FullPose & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_maneuver
get_serialized_size_key(
  const maneuver::msg::FullPose & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_maneuver
max_serialized_size_key_FullPose(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace maneuver

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_maneuver
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, maneuver, msg, FullPose)();

#ifdef __cplusplus
}
#endif

#endif  // MANEUVER__MSG__DETAIL__FULL_POSE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
