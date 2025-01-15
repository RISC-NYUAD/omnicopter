// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from controller:msg/MotorSpeed.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "controller/msg/detail/motor_speed__rosidl_typesupport_introspection_c.h"
#include "controller/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "controller/msg/detail/motor_speed__functions.h"
#include "controller/msg/detail/motor_speed__struct.h"


// Include directives for member types
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `velocity`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__MotorSpeed_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  controller__msg__MotorSpeed__init(message_memory);
}

void controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__MotorSpeed_fini_function(void * message_memory)
{
  controller__msg__MotorSpeed__fini(message_memory);
}

size_t controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__size_function__MotorSpeed__name(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__get_const_function__MotorSpeed__name(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__get_function__MotorSpeed__name(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__fetch_function__MotorSpeed__name(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__get_const_function__MotorSpeed__name(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__assign_function__MotorSpeed__name(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__get_function__MotorSpeed__name(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__resize_function__MotorSpeed__name(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__size_function__MotorSpeed__velocity(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__get_const_function__MotorSpeed__velocity(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__get_function__MotorSpeed__velocity(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__fetch_function__MotorSpeed__velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__get_const_function__MotorSpeed__velocity(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__assign_function__MotorSpeed__velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__get_function__MotorSpeed__velocity(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__resize_function__MotorSpeed__velocity(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__MotorSpeed_message_member_array[2] = {
  {
    "name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__msg__MotorSpeed, name),  // bytes offset in struct
    NULL,  // default value
    controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__size_function__MotorSpeed__name,  // size() function pointer
    controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__get_const_function__MotorSpeed__name,  // get_const(index) function pointer
    controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__get_function__MotorSpeed__name,  // get(index) function pointer
    controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__fetch_function__MotorSpeed__name,  // fetch(index, &value) function pointer
    controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__assign_function__MotorSpeed__name,  // assign(index, value) function pointer
    controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__resize_function__MotorSpeed__name  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__msg__MotorSpeed, velocity),  // bytes offset in struct
    NULL,  // default value
    controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__size_function__MotorSpeed__velocity,  // size() function pointer
    controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__get_const_function__MotorSpeed__velocity,  // get_const(index) function pointer
    controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__get_function__MotorSpeed__velocity,  // get(index) function pointer
    controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__fetch_function__MotorSpeed__velocity,  // fetch(index, &value) function pointer
    controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__assign_function__MotorSpeed__velocity,  // assign(index, value) function pointer
    controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__resize_function__MotorSpeed__velocity  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__MotorSpeed_message_members = {
  "controller__msg",  // message namespace
  "MotorSpeed",  // message name
  2,  // number of fields
  sizeof(controller__msg__MotorSpeed),
  false,  // has_any_key_member_
  controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__MotorSpeed_message_member_array,  // message members
  controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__MotorSpeed_init_function,  // function to initialize message memory (memory has to be allocated)
  controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__MotorSpeed_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__MotorSpeed_message_type_support_handle = {
  0,
  &controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__MotorSpeed_message_members,
  get_message_typesupport_handle_function,
  &controller__msg__MotorSpeed__get_type_hash,
  &controller__msg__MotorSpeed__get_type_description,
  &controller__msg__MotorSpeed__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_controller
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, msg, MotorSpeed)() {
  if (!controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__MotorSpeed_message_type_support_handle.typesupport_identifier) {
    controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__MotorSpeed_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &controller__msg__MotorSpeed__rosidl_typesupport_introspection_c__MotorSpeed_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
