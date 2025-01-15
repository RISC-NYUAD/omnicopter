// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:srv/LinearWrench.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "controller/srv/linear_wrench.h"


#ifndef CONTROLLER__SRV__DETAIL__LINEAR_WRENCH__STRUCT_H_
#define CONTROLLER__SRV__DETAIL__LINEAR_WRENCH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/LinearWrench in the package controller.
typedef struct controller__srv__LinearWrench_Request
{
  float fx1;
  float fy1;
  float fz1;
  float fx2;
  float fy2;
  float fz2;
  float ramp;
  float duration;
} controller__srv__LinearWrench_Request;

// Struct for a sequence of controller__srv__LinearWrench_Request.
typedef struct controller__srv__LinearWrench_Request__Sequence
{
  controller__srv__LinearWrench_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__LinearWrench_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/LinearWrench in the package controller.
typedef struct controller__srv__LinearWrench_Response
{
  bool status;
} controller__srv__LinearWrench_Response;

// Struct for a sequence of controller__srv__LinearWrench_Response.
typedef struct controller__srv__LinearWrench_Response__Sequence
{
  controller__srv__LinearWrench_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__LinearWrench_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  controller__srv__LinearWrench_Event__request__MAX_SIZE = 1
};
// response
enum
{
  controller__srv__LinearWrench_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/LinearWrench in the package controller.
typedef struct controller__srv__LinearWrench_Event
{
  service_msgs__msg__ServiceEventInfo info;
  controller__srv__LinearWrench_Request__Sequence request;
  controller__srv__LinearWrench_Response__Sequence response;
} controller__srv__LinearWrench_Event;

// Struct for a sequence of controller__srv__LinearWrench_Event.
typedef struct controller__srv__LinearWrench_Event__Sequence
{
  controller__srv__LinearWrench_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__LinearWrench_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__SRV__DETAIL__LINEAR_WRENCH__STRUCT_H_
