// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:msg/MotorSpeed.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "controller/msg/motor_speed.h"


#ifndef CONTROLLER__MSG__DETAIL__MOTOR_SPEED__STRUCT_H_
#define CONTROLLER__MSG__DETAIL__MOTOR_SPEED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'name'
#include "rosidl_runtime_c/string.h"
// Member 'velocity'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/MotorSpeed in the package controller.
typedef struct controller__msg__MotorSpeed
{
  rosidl_runtime_c__String__Sequence name;
  rosidl_runtime_c__float__Sequence velocity;
} controller__msg__MotorSpeed;

// Struct for a sequence of controller__msg__MotorSpeed.
typedef struct controller__msg__MotorSpeed__Sequence
{
  controller__msg__MotorSpeed * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__msg__MotorSpeed__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__MSG__DETAIL__MOTOR_SPEED__STRUCT_H_
