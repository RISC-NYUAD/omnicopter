// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from controller:msg/ErrorMsg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "controller/msg/detail/error_msg__rosidl_typesupport_introspection_c.h"
#include "controller/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "controller/msg/detail/error_msg__functions.h"
#include "controller/msg/detail/error_msg__struct.h"


// Include directives for member types
// Member `ex`
// Member `ev`
// Member `ea`
// Member `er`
// Member `ew`
// Member `iex`
// Member `ier`
// Member `accd`
#include "geometry_msgs/msg/vector3.h"
// Member `ex`
// Member `ev`
// Member `ea`
// Member `er`
// Member `ew`
// Member `iex`
// Member `ier`
// Member `accd`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"
// Member `wrench`
// Member `exwrench`
// Member `prop`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__ErrorMsg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  controller__msg__ErrorMsg__init(message_memory);
}

void controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__ErrorMsg_fini_function(void * message_memory)
{
  controller__msg__ErrorMsg__fini(message_memory);
}

size_t controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__size_function__ErrorMsg__wrench(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__get_const_function__ErrorMsg__wrench(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__get_function__ErrorMsg__wrench(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__fetch_function__ErrorMsg__wrench(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__get_const_function__ErrorMsg__wrench(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__assign_function__ErrorMsg__wrench(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__get_function__ErrorMsg__wrench(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__resize_function__ErrorMsg__wrench(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__size_function__ErrorMsg__exwrench(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__get_const_function__ErrorMsg__exwrench(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__get_function__ErrorMsg__exwrench(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__fetch_function__ErrorMsg__exwrench(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__get_const_function__ErrorMsg__exwrench(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__assign_function__ErrorMsg__exwrench(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__get_function__ErrorMsg__exwrench(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__resize_function__ErrorMsg__exwrench(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__size_function__ErrorMsg__prop(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__get_const_function__ErrorMsg__prop(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__get_function__ErrorMsg__prop(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__fetch_function__ErrorMsg__prop(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__get_const_function__ErrorMsg__prop(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__assign_function__ErrorMsg__prop(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__get_function__ErrorMsg__prop(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__resize_function__ErrorMsg__prop(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__ErrorMsg_message_member_array[12] = {
  {
    "ex",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__msg__ErrorMsg, ex),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ev",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__msg__ErrorMsg, ev),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ea",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__msg__ErrorMsg, ea),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "er",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__msg__ErrorMsg, er),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ew",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__msg__ErrorMsg, ew),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "iex",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__msg__ErrorMsg, iex),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ier",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__msg__ErrorMsg, ier),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "accd",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__msg__ErrorMsg, accd),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "wrench",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__msg__ErrorMsg, wrench),  // bytes offset in struct
    NULL,  // default value
    controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__size_function__ErrorMsg__wrench,  // size() function pointer
    controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__get_const_function__ErrorMsg__wrench,  // get_const(index) function pointer
    controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__get_function__ErrorMsg__wrench,  // get(index) function pointer
    controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__fetch_function__ErrorMsg__wrench,  // fetch(index, &value) function pointer
    controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__assign_function__ErrorMsg__wrench,  // assign(index, value) function pointer
    controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__resize_function__ErrorMsg__wrench  // resize(index) function pointer
  },
  {
    "exwrench",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__msg__ErrorMsg, exwrench),  // bytes offset in struct
    NULL,  // default value
    controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__size_function__ErrorMsg__exwrench,  // size() function pointer
    controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__get_const_function__ErrorMsg__exwrench,  // get_const(index) function pointer
    controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__get_function__ErrorMsg__exwrench,  // get(index) function pointer
    controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__fetch_function__ErrorMsg__exwrench,  // fetch(index, &value) function pointer
    controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__assign_function__ErrorMsg__exwrench,  // assign(index, value) function pointer
    controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__resize_function__ErrorMsg__exwrench  // resize(index) function pointer
  },
  {
    "prop",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__msg__ErrorMsg, prop),  // bytes offset in struct
    NULL,  // default value
    controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__size_function__ErrorMsg__prop,  // size() function pointer
    controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__get_const_function__ErrorMsg__prop,  // get_const(index) function pointer
    controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__get_function__ErrorMsg__prop,  // get(index) function pointer
    controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__fetch_function__ErrorMsg__prop,  // fetch(index, &value) function pointer
    controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__assign_function__ErrorMsg__prop,  // assign(index, value) function pointer
    controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__resize_function__ErrorMsg__prop  // resize(index) function pointer
  },
  {
    "weight",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__msg__ErrorMsg, weight),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__ErrorMsg_message_members = {
  "controller__msg",  // message namespace
  "ErrorMsg",  // message name
  12,  // number of fields
  sizeof(controller__msg__ErrorMsg),
  false,  // has_any_key_member_
  controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__ErrorMsg_message_member_array,  // message members
  controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__ErrorMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__ErrorMsg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__ErrorMsg_message_type_support_handle = {
  0,
  &controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__ErrorMsg_message_members,
  get_message_typesupport_handle_function,
  &controller__msg__ErrorMsg__get_type_hash,
  &controller__msg__ErrorMsg__get_type_description,
  &controller__msg__ErrorMsg__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_controller
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, msg, ErrorMsg)() {
  controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__ErrorMsg_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__ErrorMsg_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__ErrorMsg_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__ErrorMsg_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__ErrorMsg_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__ErrorMsg_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__ErrorMsg_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__ErrorMsg_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  if (!controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__ErrorMsg_message_type_support_handle.typesupport_identifier) {
    controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__ErrorMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &controller__msg__ErrorMsg__rosidl_typesupport_introspection_c__ErrorMsg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
