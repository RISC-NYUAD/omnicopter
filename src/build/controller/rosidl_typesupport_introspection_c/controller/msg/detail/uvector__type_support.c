// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from controller:msg/Uvector.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "controller/msg/detail/uvector__rosidl_typesupport_introspection_c.h"
#include "controller/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "controller/msg/detail/uvector__functions.h"
#include "controller/msg/detail/uvector__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void controller__msg__Uvector__rosidl_typesupport_introspection_c__Uvector_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  controller__msg__Uvector__init(message_memory);
}

void controller__msg__Uvector__rosidl_typesupport_introspection_c__Uvector_fini_function(void * message_memory)
{
  controller__msg__Uvector__fini(message_memory);
}

size_t controller__msg__Uvector__rosidl_typesupport_introspection_c__size_function__Uvector__value(
  const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * controller__msg__Uvector__rosidl_typesupport_introspection_c__get_const_function__Uvector__value(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * controller__msg__Uvector__rosidl_typesupport_introspection_c__get_function__Uvector__value(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void controller__msg__Uvector__rosidl_typesupport_introspection_c__fetch_function__Uvector__value(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    controller__msg__Uvector__rosidl_typesupport_introspection_c__get_const_function__Uvector__value(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void controller__msg__Uvector__rosidl_typesupport_introspection_c__assign_function__Uvector__value(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    controller__msg__Uvector__rosidl_typesupport_introspection_c__get_function__Uvector__value(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember controller__msg__Uvector__rosidl_typesupport_introspection_c__Uvector_message_member_array[1] = {
  {
    "value",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(controller__msg__Uvector, value),  // bytes offset in struct
    NULL,  // default value
    controller__msg__Uvector__rosidl_typesupport_introspection_c__size_function__Uvector__value,  // size() function pointer
    controller__msg__Uvector__rosidl_typesupport_introspection_c__get_const_function__Uvector__value,  // get_const(index) function pointer
    controller__msg__Uvector__rosidl_typesupport_introspection_c__get_function__Uvector__value,  // get(index) function pointer
    controller__msg__Uvector__rosidl_typesupport_introspection_c__fetch_function__Uvector__value,  // fetch(index, &value) function pointer
    controller__msg__Uvector__rosidl_typesupport_introspection_c__assign_function__Uvector__value,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers controller__msg__Uvector__rosidl_typesupport_introspection_c__Uvector_message_members = {
  "controller__msg",  // message namespace
  "Uvector",  // message name
  1,  // number of fields
  sizeof(controller__msg__Uvector),
  false,  // has_any_key_member_
  controller__msg__Uvector__rosidl_typesupport_introspection_c__Uvector_message_member_array,  // message members
  controller__msg__Uvector__rosidl_typesupport_introspection_c__Uvector_init_function,  // function to initialize message memory (memory has to be allocated)
  controller__msg__Uvector__rosidl_typesupport_introspection_c__Uvector_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t controller__msg__Uvector__rosidl_typesupport_introspection_c__Uvector_message_type_support_handle = {
  0,
  &controller__msg__Uvector__rosidl_typesupport_introspection_c__Uvector_message_members,
  get_message_typesupport_handle_function,
  &controller__msg__Uvector__get_type_hash,
  &controller__msg__Uvector__get_type_description,
  &controller__msg__Uvector__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_controller
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, msg, Uvector)() {
  if (!controller__msg__Uvector__rosidl_typesupport_introspection_c__Uvector_message_type_support_handle.typesupport_identifier) {
    controller__msg__Uvector__rosidl_typesupport_introspection_c__Uvector_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &controller__msg__Uvector__rosidl_typesupport_introspection_c__Uvector_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
