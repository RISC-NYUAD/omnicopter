// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from maneuver:msg/FullPose.idl
// generated code does not contain a copyright notice
#ifndef MANEUVER__MSG__DETAIL__FULL_POSE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define MANEUVER__MSG__DETAIL__FULL_POSE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "maneuver/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "maneuver/msg/detail/full_pose__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_maneuver
bool cdr_serialize_maneuver__msg__FullPose(
  const maneuver__msg__FullPose * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_maneuver
bool cdr_deserialize_maneuver__msg__FullPose(
  eprosima::fastcdr::Cdr &,
  maneuver__msg__FullPose * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_maneuver
size_t get_serialized_size_maneuver__msg__FullPose(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_maneuver
size_t max_serialized_size_maneuver__msg__FullPose(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_maneuver
bool cdr_serialize_key_maneuver__msg__FullPose(
  const maneuver__msg__FullPose * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_maneuver
size_t get_serialized_size_key_maneuver__msg__FullPose(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_maneuver
size_t max_serialized_size_key_maneuver__msg__FullPose(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_maneuver
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, maneuver, msg, FullPose)();

#ifdef __cplusplus
}
#endif

#endif  // MANEUVER__MSG__DETAIL__FULL_POSE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
