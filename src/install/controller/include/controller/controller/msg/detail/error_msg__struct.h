// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:msg/ErrorMsg.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "controller/msg/error_msg.h"


#ifndef CONTROLLER__MSG__DETAIL__ERROR_MSG__STRUCT_H_
#define CONTROLLER__MSG__DETAIL__ERROR_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'ex'
// Member 'ev'
// Member 'ea'
// Member 'er'
// Member 'ew'
// Member 'iex'
// Member 'ier'
// Member 'accd'
#include "geometry_msgs/msg/detail/vector3__struct.h"
// Member 'wrench'
// Member 'exwrench'
// Member 'prop'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/ErrorMsg in the package controller.
typedef struct controller__msg__ErrorMsg
{
  geometry_msgs__msg__Vector3 ex;
  geometry_msgs__msg__Vector3 ev;
  geometry_msgs__msg__Vector3 ea;
  geometry_msgs__msg__Vector3 er;
  geometry_msgs__msg__Vector3 ew;
  geometry_msgs__msg__Vector3 iex;
  geometry_msgs__msg__Vector3 ier;
  geometry_msgs__msg__Vector3 accd;
  rosidl_runtime_c__float__Sequence wrench;
  rosidl_runtime_c__float__Sequence exwrench;
  rosidl_runtime_c__float__Sequence prop;
  float weight;
} controller__msg__ErrorMsg;

// Struct for a sequence of controller__msg__ErrorMsg.
typedef struct controller__msg__ErrorMsg__Sequence
{
  controller__msg__ErrorMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__msg__ErrorMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__MSG__DETAIL__ERROR_MSG__STRUCT_H_
