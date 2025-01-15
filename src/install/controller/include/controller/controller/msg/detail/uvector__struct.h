// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:msg/Uvector.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "controller/msg/uvector.h"


#ifndef CONTROLLER__MSG__DETAIL__UVECTOR__STRUCT_H_
#define CONTROLLER__MSG__DETAIL__UVECTOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/Uvector in the package controller.
typedef struct controller__msg__Uvector
{
  float value[12];
} controller__msg__Uvector;

// Struct for a sequence of controller__msg__Uvector.
typedef struct controller__msg__Uvector__Sequence
{
  controller__msg__Uvector * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__msg__Uvector__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__MSG__DETAIL__UVECTOR__STRUCT_H_
