// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:srv/AngularWrench.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "controller/srv/angular_wrench.h"


#ifndef CONTROLLER__SRV__DETAIL__ANGULAR_WRENCH__STRUCT_H_
#define CONTROLLER__SRV__DETAIL__ANGULAR_WRENCH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/AngularWrench in the package controller.
typedef struct controller__srv__AngularWrench_Request
{
  float fz;
  float phi1;
  float phi2;
  float duration;
} controller__srv__AngularWrench_Request;

// Struct for a sequence of controller__srv__AngularWrench_Request.
typedef struct controller__srv__AngularWrench_Request__Sequence
{
  controller__srv__AngularWrench_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__AngularWrench_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/AngularWrench in the package controller.
typedef struct controller__srv__AngularWrench_Response
{
  bool status;
} controller__srv__AngularWrench_Response;

// Struct for a sequence of controller__srv__AngularWrench_Response.
typedef struct controller__srv__AngularWrench_Response__Sequence
{
  controller__srv__AngularWrench_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__AngularWrench_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  controller__srv__AngularWrench_Event__request__MAX_SIZE = 1
};
// response
enum
{
  controller__srv__AngularWrench_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/AngularWrench in the package controller.
typedef struct controller__srv__AngularWrench_Event
{
  service_msgs__msg__ServiceEventInfo info;
  controller__srv__AngularWrench_Request__Sequence request;
  controller__srv__AngularWrench_Response__Sequence response;
} controller__srv__AngularWrench_Event;

// Struct for a sequence of controller__srv__AngularWrench_Event.
typedef struct controller__srv__AngularWrench_Event__Sequence
{
  controller__srv__AngularWrench_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__AngularWrench_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__SRV__DETAIL__ANGULAR_WRENCH__STRUCT_H_
