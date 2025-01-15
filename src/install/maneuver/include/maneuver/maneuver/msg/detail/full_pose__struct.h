// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from maneuver:msg/FullPose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "maneuver/msg/full_pose.h"


#ifndef MANEUVER__MSG__DETAIL__FULL_POSE__STRUCT_H_
#define MANEUVER__MSG__DETAIL__FULL_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'vel'
#include "geometry_msgs/msg/detail/twist__struct.h"
// Member 'acc'
#include "geometry_msgs/msg/detail/accel__struct.h"

/// Struct defined in msg/FullPose in the package maneuver.
typedef struct maneuver__msg__FullPose
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Pose pose;
  geometry_msgs__msg__Twist vel;
  geometry_msgs__msg__Accel acc;
} maneuver__msg__FullPose;

// Struct for a sequence of maneuver__msg__FullPose.
typedef struct maneuver__msg__FullPose__Sequence
{
  maneuver__msg__FullPose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} maneuver__msg__FullPose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MANEUVER__MSG__DETAIL__FULL_POSE__STRUCT_H_
