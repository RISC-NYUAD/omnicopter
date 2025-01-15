// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from controller:msg/Pose.idl
// generated code does not contain a copyright notice
#ifndef CONTROLLER__MSG__DETAIL__POSE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define CONTROLLER__MSG__DETAIL__POSE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "controller/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "controller/msg/detail/pose__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_controller
bool cdr_serialize_controller__msg__Pose(
  const controller__msg__Pose * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_controller
bool cdr_deserialize_controller__msg__Pose(
  eprosima::fastcdr::Cdr &,
  controller__msg__Pose * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_controller
size_t get_serialized_size_controller__msg__Pose(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_controller
size_t max_serialized_size_controller__msg__Pose(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_controller
bool cdr_serialize_key_controller__msg__Pose(
  const controller__msg__Pose * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_controller
size_t get_serialized_size_key_controller__msg__Pose(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_controller
size_t max_serialized_size_key_controller__msg__Pose(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_controller
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, controller, msg, Pose)();

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__MSG__DETAIL__POSE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
