// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from maneuver:srv/Ellipse5D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "maneuver/srv/ellipse5_d.h"


#ifndef MANEUVER__SRV__DETAIL__ELLIPSE5_D__STRUCT_H_
#define MANEUVER__SRV__DETAIL__ELLIPSE5_D__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Ellipse5D in the package maneuver.
typedef struct maneuver__srv__Ellipse5D_Request
{
  float x_min;
  float x_max;
  float y_min;
  float y_max;
  float z_min;
  float z_max;
  float roll_start;
  float roll_mid;
  float roll_end;
  float pitch_start;
  float pitch_mid;
  float pitch_end;
  float yaw;
  float duration;
} maneuver__srv__Ellipse5D_Request;

// Struct for a sequence of maneuver__srv__Ellipse5D_Request.
typedef struct maneuver__srv__Ellipse5D_Request__Sequence
{
  maneuver__srv__Ellipse5D_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} maneuver__srv__Ellipse5D_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/Ellipse5D in the package maneuver.
typedef struct maneuver__srv__Ellipse5D_Response
{
  bool status;
} maneuver__srv__Ellipse5D_Response;

// Struct for a sequence of maneuver__srv__Ellipse5D_Response.
typedef struct maneuver__srv__Ellipse5D_Response__Sequence
{
  maneuver__srv__Ellipse5D_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} maneuver__srv__Ellipse5D_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  maneuver__srv__Ellipse5D_Event__request__MAX_SIZE = 1
};
// response
enum
{
  maneuver__srv__Ellipse5D_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/Ellipse5D in the package maneuver.
typedef struct maneuver__srv__Ellipse5D_Event
{
  service_msgs__msg__ServiceEventInfo info;
  maneuver__srv__Ellipse5D_Request__Sequence request;
  maneuver__srv__Ellipse5D_Response__Sequence response;
} maneuver__srv__Ellipse5D_Event;

// Struct for a sequence of maneuver__srv__Ellipse5D_Event.
typedef struct maneuver__srv__Ellipse5D_Event__Sequence
{
  maneuver__srv__Ellipse5D_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} maneuver__srv__Ellipse5D_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MANEUVER__SRV__DETAIL__ELLIPSE5_D__STRUCT_H_
