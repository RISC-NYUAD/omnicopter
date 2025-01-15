// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from controller:srv/LinearWrench.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "controller/srv/detail/linear_wrench__rosidl_typesupport_introspection_c.h"
#include "controller/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "controller/srv/detail/linear_wrench__functions.h"
#include "controller/srv/detail/linear_wrench__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void controller__srv__LinearWrench_Request__rosidl_typesupport_introspection_c__LinearWrench_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  controller__srv__LinearWrench_Request__init(message_memory);
}

void controller__srv__LinearWrench_Request__rosidl_typesupport_introspection_c__LinearWrench_Request_fini_function(void * message_memory)
{
  controller__srv__LinearWrench_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember controller__srv__LinearWrench_Request__rosidl_typesupport_introspection_c__LinearWrench_Request_message_member_array[8] = {
  {
    "fx1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__srv__LinearWrench_Request, fx1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "fy1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__srv__LinearWrench_Request, fy1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "fz1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__srv__LinearWrench_Request, fz1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "fx2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__srv__LinearWrench_Request, fx2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "fy2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__srv__LinearWrench_Request, fy2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "fz2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__srv__LinearWrench_Request, fz2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ramp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__srv__LinearWrench_Request, ramp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "duration",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__srv__LinearWrench_Request, duration),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers controller__srv__LinearWrench_Request__rosidl_typesupport_introspection_c__LinearWrench_Request_message_members = {
  "controller__srv",  // message namespace
  "LinearWrench_Request",  // message name
  8,  // number of fields
  sizeof(controller__srv__LinearWrench_Request),
  false,  // has_any_key_member_
  controller__srv__LinearWrench_Request__rosidl_typesupport_introspection_c__LinearWrench_Request_message_member_array,  // message members
  controller__srv__LinearWrench_Request__rosidl_typesupport_introspection_c__LinearWrench_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  controller__srv__LinearWrench_Request__rosidl_typesupport_introspection_c__LinearWrench_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t controller__srv__LinearWrench_Request__rosidl_typesupport_introspection_c__LinearWrench_Request_message_type_support_handle = {
  0,
  &controller__srv__LinearWrench_Request__rosidl_typesupport_introspection_c__LinearWrench_Request_message_members,
  get_message_typesupport_handle_function,
  &controller__srv__LinearWrench_Request__get_type_hash,
  &controller__srv__LinearWrench_Request__get_type_description,
  &controller__srv__LinearWrench_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_controller
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, srv, LinearWrench_Request)() {
  if (!controller__srv__LinearWrench_Request__rosidl_typesupport_introspection_c__LinearWrench_Request_message_type_support_handle.typesupport_identifier) {
    controller__srv__LinearWrench_Request__rosidl_typesupport_introspection_c__LinearWrench_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &controller__srv__LinearWrench_Request__rosidl_typesupport_introspection_c__LinearWrench_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "controller/srv/detail/linear_wrench__rosidl_typesupport_introspection_c.h"
// already included above
// #include "controller/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "controller/srv/detail/linear_wrench__functions.h"
// already included above
// #include "controller/srv/detail/linear_wrench__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void controller__srv__LinearWrench_Response__rosidl_typesupport_introspection_c__LinearWrench_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  controller__srv__LinearWrench_Response__init(message_memory);
}

void controller__srv__LinearWrench_Response__rosidl_typesupport_introspection_c__LinearWrench_Response_fini_function(void * message_memory)
{
  controller__srv__LinearWrench_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember controller__srv__LinearWrench_Response__rosidl_typesupport_introspection_c__LinearWrench_Response_message_member_array[1] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__srv__LinearWrench_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers controller__srv__LinearWrench_Response__rosidl_typesupport_introspection_c__LinearWrench_Response_message_members = {
  "controller__srv",  // message namespace
  "LinearWrench_Response",  // message name
  1,  // number of fields
  sizeof(controller__srv__LinearWrench_Response),
  false,  // has_any_key_member_
  controller__srv__LinearWrench_Response__rosidl_typesupport_introspection_c__LinearWrench_Response_message_member_array,  // message members
  controller__srv__LinearWrench_Response__rosidl_typesupport_introspection_c__LinearWrench_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  controller__srv__LinearWrench_Response__rosidl_typesupport_introspection_c__LinearWrench_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t controller__srv__LinearWrench_Response__rosidl_typesupport_introspection_c__LinearWrench_Response_message_type_support_handle = {
  0,
  &controller__srv__LinearWrench_Response__rosidl_typesupport_introspection_c__LinearWrench_Response_message_members,
  get_message_typesupport_handle_function,
  &controller__srv__LinearWrench_Response__get_type_hash,
  &controller__srv__LinearWrench_Response__get_type_description,
  &controller__srv__LinearWrench_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_controller
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, srv, LinearWrench_Response)() {
  if (!controller__srv__LinearWrench_Response__rosidl_typesupport_introspection_c__LinearWrench_Response_message_type_support_handle.typesupport_identifier) {
    controller__srv__LinearWrench_Response__rosidl_typesupport_introspection_c__LinearWrench_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &controller__srv__LinearWrench_Response__rosidl_typesupport_introspection_c__LinearWrench_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "controller/srv/detail/linear_wrench__rosidl_typesupport_introspection_c.h"
// already included above
// #include "controller/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "controller/srv/detail/linear_wrench__functions.h"
// already included above
// #include "controller/srv/detail/linear_wrench__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "controller/srv/linear_wrench.h"
// Member `request`
// Member `response`
// already included above
// #include "controller/srv/detail/linear_wrench__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__LinearWrench_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  controller__srv__LinearWrench_Event__init(message_memory);
}

void controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__LinearWrench_Event_fini_function(void * message_memory)
{
  controller__srv__LinearWrench_Event__fini(message_memory);
}

size_t controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__size_function__LinearWrench_Event__request(
  const void * untyped_member)
{
  const controller__srv__LinearWrench_Request__Sequence * member =
    (const controller__srv__LinearWrench_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__get_const_function__LinearWrench_Event__request(
  const void * untyped_member, size_t index)
{
  const controller__srv__LinearWrench_Request__Sequence * member =
    (const controller__srv__LinearWrench_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__get_function__LinearWrench_Event__request(
  void * untyped_member, size_t index)
{
  controller__srv__LinearWrench_Request__Sequence * member =
    (controller__srv__LinearWrench_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__fetch_function__LinearWrench_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const controller__srv__LinearWrench_Request * item =
    ((const controller__srv__LinearWrench_Request *)
    controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__get_const_function__LinearWrench_Event__request(untyped_member, index));
  controller__srv__LinearWrench_Request * value =
    (controller__srv__LinearWrench_Request *)(untyped_value);
  *value = *item;
}

void controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__assign_function__LinearWrench_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  controller__srv__LinearWrench_Request * item =
    ((controller__srv__LinearWrench_Request *)
    controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__get_function__LinearWrench_Event__request(untyped_member, index));
  const controller__srv__LinearWrench_Request * value =
    (const controller__srv__LinearWrench_Request *)(untyped_value);
  *item = *value;
}

bool controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__resize_function__LinearWrench_Event__request(
  void * untyped_member, size_t size)
{
  controller__srv__LinearWrench_Request__Sequence * member =
    (controller__srv__LinearWrench_Request__Sequence *)(untyped_member);
  controller__srv__LinearWrench_Request__Sequence__fini(member);
  return controller__srv__LinearWrench_Request__Sequence__init(member, size);
}

size_t controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__size_function__LinearWrench_Event__response(
  const void * untyped_member)
{
  const controller__srv__LinearWrench_Response__Sequence * member =
    (const controller__srv__LinearWrench_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__get_const_function__LinearWrench_Event__response(
  const void * untyped_member, size_t index)
{
  const controller__srv__LinearWrench_Response__Sequence * member =
    (const controller__srv__LinearWrench_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__get_function__LinearWrench_Event__response(
  void * untyped_member, size_t index)
{
  controller__srv__LinearWrench_Response__Sequence * member =
    (controller__srv__LinearWrench_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__fetch_function__LinearWrench_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const controller__srv__LinearWrench_Response * item =
    ((const controller__srv__LinearWrench_Response *)
    controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__get_const_function__LinearWrench_Event__response(untyped_member, index));
  controller__srv__LinearWrench_Response * value =
    (controller__srv__LinearWrench_Response *)(untyped_value);
  *value = *item;
}

void controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__assign_function__LinearWrench_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  controller__srv__LinearWrench_Response * item =
    ((controller__srv__LinearWrench_Response *)
    controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__get_function__LinearWrench_Event__response(untyped_member, index));
  const controller__srv__LinearWrench_Response * value =
    (const controller__srv__LinearWrench_Response *)(untyped_value);
  *item = *value;
}

bool controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__resize_function__LinearWrench_Event__response(
  void * untyped_member, size_t size)
{
  controller__srv__LinearWrench_Response__Sequence * member =
    (controller__srv__LinearWrench_Response__Sequence *)(untyped_member);
  controller__srv__LinearWrench_Response__Sequence__fini(member);
  return controller__srv__LinearWrench_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__LinearWrench_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__srv__LinearWrench_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(controller__srv__LinearWrench_Event, request),  // bytes offset in struct
    NULL,  // default value
    controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__size_function__LinearWrench_Event__request,  // size() function pointer
    controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__get_const_function__LinearWrench_Event__request,  // get_const(index) function pointer
    controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__get_function__LinearWrench_Event__request,  // get(index) function pointer
    controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__fetch_function__LinearWrench_Event__request,  // fetch(index, &value) function pointer
    controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__assign_function__LinearWrench_Event__request,  // assign(index, value) function pointer
    controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__resize_function__LinearWrench_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(controller__srv__LinearWrench_Event, response),  // bytes offset in struct
    NULL,  // default value
    controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__size_function__LinearWrench_Event__response,  // size() function pointer
    controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__get_const_function__LinearWrench_Event__response,  // get_const(index) function pointer
    controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__get_function__LinearWrench_Event__response,  // get(index) function pointer
    controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__fetch_function__LinearWrench_Event__response,  // fetch(index, &value) function pointer
    controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__assign_function__LinearWrench_Event__response,  // assign(index, value) function pointer
    controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__resize_function__LinearWrench_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__LinearWrench_Event_message_members = {
  "controller__srv",  // message namespace
  "LinearWrench_Event",  // message name
  3,  // number of fields
  sizeof(controller__srv__LinearWrench_Event),
  false,  // has_any_key_member_
  controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__LinearWrench_Event_message_member_array,  // message members
  controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__LinearWrench_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__LinearWrench_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__LinearWrench_Event_message_type_support_handle = {
  0,
  &controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__LinearWrench_Event_message_members,
  get_message_typesupport_handle_function,
  &controller__srv__LinearWrench_Event__get_type_hash,
  &controller__srv__LinearWrench_Event__get_type_description,
  &controller__srv__LinearWrench_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_controller
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, srv, LinearWrench_Event)() {
  controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__LinearWrench_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__LinearWrench_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, srv, LinearWrench_Request)();
  controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__LinearWrench_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, srv, LinearWrench_Response)();
  if (!controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__LinearWrench_Event_message_type_support_handle.typesupport_identifier) {
    controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__LinearWrench_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__LinearWrench_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "controller/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "controller/srv/detail/linear_wrench__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers controller__srv__detail__linear_wrench__rosidl_typesupport_introspection_c__LinearWrench_service_members = {
  "controller__srv",  // service namespace
  "LinearWrench",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // controller__srv__detail__linear_wrench__rosidl_typesupport_introspection_c__LinearWrench_Request_message_type_support_handle,
  NULL,  // response message
  // controller__srv__detail__linear_wrench__rosidl_typesupport_introspection_c__LinearWrench_Response_message_type_support_handle
  NULL  // event_message
  // controller__srv__detail__linear_wrench__rosidl_typesupport_introspection_c__LinearWrench_Response_message_type_support_handle
};


static rosidl_service_type_support_t controller__srv__detail__linear_wrench__rosidl_typesupport_introspection_c__LinearWrench_service_type_support_handle = {
  0,
  &controller__srv__detail__linear_wrench__rosidl_typesupport_introspection_c__LinearWrench_service_members,
  get_service_typesupport_handle_function,
  &controller__srv__LinearWrench_Request__rosidl_typesupport_introspection_c__LinearWrench_Request_message_type_support_handle,
  &controller__srv__LinearWrench_Response__rosidl_typesupport_introspection_c__LinearWrench_Response_message_type_support_handle,
  &controller__srv__LinearWrench_Event__rosidl_typesupport_introspection_c__LinearWrench_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    controller,
    srv,
    LinearWrench
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    controller,
    srv,
    LinearWrench
  ),
  &controller__srv__LinearWrench__get_type_hash,
  &controller__srv__LinearWrench__get_type_description,
  &controller__srv__LinearWrench__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, srv, LinearWrench_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, srv, LinearWrench_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, srv, LinearWrench_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_controller
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, srv, LinearWrench)(void) {
  if (!controller__srv__detail__linear_wrench__rosidl_typesupport_introspection_c__LinearWrench_service_type_support_handle.typesupport_identifier) {
    controller__srv__detail__linear_wrench__rosidl_typesupport_introspection_c__LinearWrench_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)controller__srv__detail__linear_wrench__rosidl_typesupport_introspection_c__LinearWrench_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, srv, LinearWrench_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, srv, LinearWrench_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, srv, LinearWrench_Event)()->data;
  }

  return &controller__srv__detail__linear_wrench__rosidl_typesupport_introspection_c__LinearWrench_service_type_support_handle;
}
