// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from maneuver:srv/Land.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "maneuver/srv/detail/land__rosidl_typesupport_introspection_c.h"
#include "maneuver/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "maneuver/srv/detail/land__functions.h"
#include "maneuver/srv/detail/land__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void maneuver__srv__Land_Request__rosidl_typesupport_introspection_c__Land_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  maneuver__srv__Land_Request__init(message_memory);
}

void maneuver__srv__Land_Request__rosidl_typesupport_introspection_c__Land_Request_fini_function(void * message_memory)
{
  maneuver__srv__Land_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember maneuver__srv__Land_Request__rosidl_typesupport_introspection_c__Land_Request_message_member_array[4] = {
  {
    "height_1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(maneuver__srv__Land_Request, height_1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "duration_1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(maneuver__srv__Land_Request, duration_1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "height_2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(maneuver__srv__Land_Request, height_2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "duration_2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(maneuver__srv__Land_Request, duration_2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers maneuver__srv__Land_Request__rosidl_typesupport_introspection_c__Land_Request_message_members = {
  "maneuver__srv",  // message namespace
  "Land_Request",  // message name
  4,  // number of fields
  sizeof(maneuver__srv__Land_Request),
  false,  // has_any_key_member_
  maneuver__srv__Land_Request__rosidl_typesupport_introspection_c__Land_Request_message_member_array,  // message members
  maneuver__srv__Land_Request__rosidl_typesupport_introspection_c__Land_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  maneuver__srv__Land_Request__rosidl_typesupport_introspection_c__Land_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t maneuver__srv__Land_Request__rosidl_typesupport_introspection_c__Land_Request_message_type_support_handle = {
  0,
  &maneuver__srv__Land_Request__rosidl_typesupport_introspection_c__Land_Request_message_members,
  get_message_typesupport_handle_function,
  &maneuver__srv__Land_Request__get_type_hash,
  &maneuver__srv__Land_Request__get_type_description,
  &maneuver__srv__Land_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_maneuver
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, maneuver, srv, Land_Request)() {
  if (!maneuver__srv__Land_Request__rosidl_typesupport_introspection_c__Land_Request_message_type_support_handle.typesupport_identifier) {
    maneuver__srv__Land_Request__rosidl_typesupport_introspection_c__Land_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &maneuver__srv__Land_Request__rosidl_typesupport_introspection_c__Land_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "maneuver/srv/detail/land__rosidl_typesupport_introspection_c.h"
// already included above
// #include "maneuver/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "maneuver/srv/detail/land__functions.h"
// already included above
// #include "maneuver/srv/detail/land__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void maneuver__srv__Land_Response__rosidl_typesupport_introspection_c__Land_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  maneuver__srv__Land_Response__init(message_memory);
}

void maneuver__srv__Land_Response__rosidl_typesupport_introspection_c__Land_Response_fini_function(void * message_memory)
{
  maneuver__srv__Land_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember maneuver__srv__Land_Response__rosidl_typesupport_introspection_c__Land_Response_message_member_array[1] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(maneuver__srv__Land_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers maneuver__srv__Land_Response__rosidl_typesupport_introspection_c__Land_Response_message_members = {
  "maneuver__srv",  // message namespace
  "Land_Response",  // message name
  1,  // number of fields
  sizeof(maneuver__srv__Land_Response),
  false,  // has_any_key_member_
  maneuver__srv__Land_Response__rosidl_typesupport_introspection_c__Land_Response_message_member_array,  // message members
  maneuver__srv__Land_Response__rosidl_typesupport_introspection_c__Land_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  maneuver__srv__Land_Response__rosidl_typesupport_introspection_c__Land_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t maneuver__srv__Land_Response__rosidl_typesupport_introspection_c__Land_Response_message_type_support_handle = {
  0,
  &maneuver__srv__Land_Response__rosidl_typesupport_introspection_c__Land_Response_message_members,
  get_message_typesupport_handle_function,
  &maneuver__srv__Land_Response__get_type_hash,
  &maneuver__srv__Land_Response__get_type_description,
  &maneuver__srv__Land_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_maneuver
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, maneuver, srv, Land_Response)() {
  if (!maneuver__srv__Land_Response__rosidl_typesupport_introspection_c__Land_Response_message_type_support_handle.typesupport_identifier) {
    maneuver__srv__Land_Response__rosidl_typesupport_introspection_c__Land_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &maneuver__srv__Land_Response__rosidl_typesupport_introspection_c__Land_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "maneuver/srv/detail/land__rosidl_typesupport_introspection_c.h"
// already included above
// #include "maneuver/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "maneuver/srv/detail/land__functions.h"
// already included above
// #include "maneuver/srv/detail/land__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "maneuver/srv/land.h"
// Member `request`
// Member `response`
// already included above
// #include "maneuver/srv/detail/land__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__Land_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  maneuver__srv__Land_Event__init(message_memory);
}

void maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__Land_Event_fini_function(void * message_memory)
{
  maneuver__srv__Land_Event__fini(message_memory);
}

size_t maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__size_function__Land_Event__request(
  const void * untyped_member)
{
  const maneuver__srv__Land_Request__Sequence * member =
    (const maneuver__srv__Land_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__get_const_function__Land_Event__request(
  const void * untyped_member, size_t index)
{
  const maneuver__srv__Land_Request__Sequence * member =
    (const maneuver__srv__Land_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__get_function__Land_Event__request(
  void * untyped_member, size_t index)
{
  maneuver__srv__Land_Request__Sequence * member =
    (maneuver__srv__Land_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__fetch_function__Land_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const maneuver__srv__Land_Request * item =
    ((const maneuver__srv__Land_Request *)
    maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__get_const_function__Land_Event__request(untyped_member, index));
  maneuver__srv__Land_Request * value =
    (maneuver__srv__Land_Request *)(untyped_value);
  *value = *item;
}

void maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__assign_function__Land_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  maneuver__srv__Land_Request * item =
    ((maneuver__srv__Land_Request *)
    maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__get_function__Land_Event__request(untyped_member, index));
  const maneuver__srv__Land_Request * value =
    (const maneuver__srv__Land_Request *)(untyped_value);
  *item = *value;
}

bool maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__resize_function__Land_Event__request(
  void * untyped_member, size_t size)
{
  maneuver__srv__Land_Request__Sequence * member =
    (maneuver__srv__Land_Request__Sequence *)(untyped_member);
  maneuver__srv__Land_Request__Sequence__fini(member);
  return maneuver__srv__Land_Request__Sequence__init(member, size);
}

size_t maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__size_function__Land_Event__response(
  const void * untyped_member)
{
  const maneuver__srv__Land_Response__Sequence * member =
    (const maneuver__srv__Land_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__get_const_function__Land_Event__response(
  const void * untyped_member, size_t index)
{
  const maneuver__srv__Land_Response__Sequence * member =
    (const maneuver__srv__Land_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__get_function__Land_Event__response(
  void * untyped_member, size_t index)
{
  maneuver__srv__Land_Response__Sequence * member =
    (maneuver__srv__Land_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__fetch_function__Land_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const maneuver__srv__Land_Response * item =
    ((const maneuver__srv__Land_Response *)
    maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__get_const_function__Land_Event__response(untyped_member, index));
  maneuver__srv__Land_Response * value =
    (maneuver__srv__Land_Response *)(untyped_value);
  *value = *item;
}

void maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__assign_function__Land_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  maneuver__srv__Land_Response * item =
    ((maneuver__srv__Land_Response *)
    maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__get_function__Land_Event__response(untyped_member, index));
  const maneuver__srv__Land_Response * value =
    (const maneuver__srv__Land_Response *)(untyped_value);
  *item = *value;
}

bool maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__resize_function__Land_Event__response(
  void * untyped_member, size_t size)
{
  maneuver__srv__Land_Response__Sequence * member =
    (maneuver__srv__Land_Response__Sequence *)(untyped_member);
  maneuver__srv__Land_Response__Sequence__fini(member);
  return maneuver__srv__Land_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__Land_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(maneuver__srv__Land_Event, info),  // bytes offset in struct
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
    offsetof(maneuver__srv__Land_Event, request),  // bytes offset in struct
    NULL,  // default value
    maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__size_function__Land_Event__request,  // size() function pointer
    maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__get_const_function__Land_Event__request,  // get_const(index) function pointer
    maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__get_function__Land_Event__request,  // get(index) function pointer
    maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__fetch_function__Land_Event__request,  // fetch(index, &value) function pointer
    maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__assign_function__Land_Event__request,  // assign(index, value) function pointer
    maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__resize_function__Land_Event__request  // resize(index) function pointer
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
    offsetof(maneuver__srv__Land_Event, response),  // bytes offset in struct
    NULL,  // default value
    maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__size_function__Land_Event__response,  // size() function pointer
    maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__get_const_function__Land_Event__response,  // get_const(index) function pointer
    maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__get_function__Land_Event__response,  // get(index) function pointer
    maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__fetch_function__Land_Event__response,  // fetch(index, &value) function pointer
    maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__assign_function__Land_Event__response,  // assign(index, value) function pointer
    maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__resize_function__Land_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__Land_Event_message_members = {
  "maneuver__srv",  // message namespace
  "Land_Event",  // message name
  3,  // number of fields
  sizeof(maneuver__srv__Land_Event),
  false,  // has_any_key_member_
  maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__Land_Event_message_member_array,  // message members
  maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__Land_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__Land_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__Land_Event_message_type_support_handle = {
  0,
  &maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__Land_Event_message_members,
  get_message_typesupport_handle_function,
  &maneuver__srv__Land_Event__get_type_hash,
  &maneuver__srv__Land_Event__get_type_description,
  &maneuver__srv__Land_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_maneuver
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, maneuver, srv, Land_Event)() {
  maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__Land_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__Land_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, maneuver, srv, Land_Request)();
  maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__Land_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, maneuver, srv, Land_Response)();
  if (!maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__Land_Event_message_type_support_handle.typesupport_identifier) {
    maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__Land_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__Land_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "maneuver/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "maneuver/srv/detail/land__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers maneuver__srv__detail__land__rosidl_typesupport_introspection_c__Land_service_members = {
  "maneuver__srv",  // service namespace
  "Land",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // maneuver__srv__detail__land__rosidl_typesupport_introspection_c__Land_Request_message_type_support_handle,
  NULL,  // response message
  // maneuver__srv__detail__land__rosidl_typesupport_introspection_c__Land_Response_message_type_support_handle
  NULL  // event_message
  // maneuver__srv__detail__land__rosidl_typesupport_introspection_c__Land_Response_message_type_support_handle
};


static rosidl_service_type_support_t maneuver__srv__detail__land__rosidl_typesupport_introspection_c__Land_service_type_support_handle = {
  0,
  &maneuver__srv__detail__land__rosidl_typesupport_introspection_c__Land_service_members,
  get_service_typesupport_handle_function,
  &maneuver__srv__Land_Request__rosidl_typesupport_introspection_c__Land_Request_message_type_support_handle,
  &maneuver__srv__Land_Response__rosidl_typesupport_introspection_c__Land_Response_message_type_support_handle,
  &maneuver__srv__Land_Event__rosidl_typesupport_introspection_c__Land_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    maneuver,
    srv,
    Land
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    maneuver,
    srv,
    Land
  ),
  &maneuver__srv__Land__get_type_hash,
  &maneuver__srv__Land__get_type_description,
  &maneuver__srv__Land__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, maneuver, srv, Land_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, maneuver, srv, Land_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, maneuver, srv, Land_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_maneuver
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, maneuver, srv, Land)(void) {
  if (!maneuver__srv__detail__land__rosidl_typesupport_introspection_c__Land_service_type_support_handle.typesupport_identifier) {
    maneuver__srv__detail__land__rosidl_typesupport_introspection_c__Land_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)maneuver__srv__detail__land__rosidl_typesupport_introspection_c__Land_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, maneuver, srv, Land_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, maneuver, srv, Land_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, maneuver, srv, Land_Event)()->data;
  }

  return &maneuver__srv__detail__land__rosidl_typesupport_introspection_c__Land_service_type_support_handle;
}
