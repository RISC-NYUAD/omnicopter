// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from maneuver:srv/Goto6DPoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "maneuver/srv/goto6_d_point.h"


#ifndef MANEUVER__SRV__DETAIL__GOTO6_D_POINT__STRUCT_H_
#define MANEUVER__SRV__DETAIL__GOTO6_D_POINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Goto6DPoint in the package maneuver.
typedef struct maneuver__srv__Goto6DPoint_Request
{
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;
  float duration;
} maneuver__srv__Goto6DPoint_Request;

// Struct for a sequence of maneuver__srv__Goto6DPoint_Request.
typedef struct maneuver__srv__Goto6DPoint_Request__Sequence
{
  maneuver__srv__Goto6DPoint_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} maneuver__srv__Goto6DPoint_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/Goto6DPoint in the package maneuver.
typedef struct maneuver__srv__Goto6DPoint_Response
{
  bool status;
} maneuver__srv__Goto6DPoint_Response;

// Struct for a sequence of maneuver__srv__Goto6DPoint_Response.
typedef struct maneuver__srv__Goto6DPoint_Response__Sequence
{
  maneuver__srv__Goto6DPoint_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} maneuver__srv__Goto6DPoint_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  maneuver__srv__Goto6DPoint_Event__request__MAX_SIZE = 1
};
// response
enum
{
  maneuver__srv__Goto6DPoint_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/Goto6DPoint in the package maneuver.
typedef struct maneuver__srv__Goto6DPoint_Event
{
  service_msgs__msg__ServiceEventInfo info;
  maneuver__srv__Goto6DPoint_Request__Sequence request;
  maneuver__srv__Goto6DPoint_Response__Sequence response;
} maneuver__srv__Goto6DPoint_Event;

// Struct for a sequence of maneuver__srv__Goto6DPoint_Event.
typedef struct maneuver__srv__Goto6DPoint_Event__Sequence
{
  maneuver__srv__Goto6DPoint_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} maneuver__srv__Goto6DPoint_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MANEUVER__SRV__DETAIL__GOTO6_D_POINT__STRUCT_H_
