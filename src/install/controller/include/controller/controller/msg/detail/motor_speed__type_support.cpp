// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from controller:msg/MotorSpeed.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "controller/msg/detail/motor_speed__functions.h"
#include "controller/msg/detail/motor_speed__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace controller
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MotorSpeed_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) controller::msg::MotorSpeed(_init);
}

void MotorSpeed_fini_function(void * message_memory)
{
  auto typed_message = static_cast<controller::msg::MotorSpeed *>(message_memory);
  typed_message->~MotorSpeed();
}

size_t size_function__MotorSpeed__name(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotorSpeed__name(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorSpeed__name(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorSpeed__name(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__MotorSpeed__name(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__MotorSpeed__name(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__MotorSpeed__name(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__MotorSpeed__name(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MotorSpeed__velocity(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotorSpeed__velocity(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorSpeed__velocity(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorSpeed__velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__MotorSpeed__velocity(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__MotorSpeed__velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__MotorSpeed__velocity(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__MotorSpeed__velocity(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MotorSpeed_message_member_array[2] = {
  {
    "name",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller::msg::MotorSpeed, name),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorSpeed__name,  // size() function pointer
    get_const_function__MotorSpeed__name,  // get_const(index) function pointer
    get_function__MotorSpeed__name,  // get(index) function pointer
    fetch_function__MotorSpeed__name,  // fetch(index, &value) function pointer
    assign_function__MotorSpeed__name,  // assign(index, value) function pointer
    resize_function__MotorSpeed__name  // resize(index) function pointer
  },
  {
    "velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller::msg::MotorSpeed, velocity),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorSpeed__velocity,  // size() function pointer
    get_const_function__MotorSpeed__velocity,  // get_const(index) function pointer
    get_function__MotorSpeed__velocity,  // get(index) function pointer
    fetch_function__MotorSpeed__velocity,  // fetch(index, &value) function pointer
    assign_function__MotorSpeed__velocity,  // assign(index, value) function pointer
    resize_function__MotorSpeed__velocity  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MotorSpeed_message_members = {
  "controller::msg",  // message namespace
  "MotorSpeed",  // message name
  2,  // number of fields
  sizeof(controller::msg::MotorSpeed),
  false,  // has_any_key_member_
  MotorSpeed_message_member_array,  // message members
  MotorSpeed_init_function,  // function to initialize message memory (memory has to be allocated)
  MotorSpeed_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MotorSpeed_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MotorSpeed_message_members,
  get_message_typesupport_handle_function,
  &controller__msg__MotorSpeed__get_type_hash,
  &controller__msg__MotorSpeed__get_type_description,
  &controller__msg__MotorSpeed__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace controller


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<controller::msg::MotorSpeed>()
{
  return &::controller::msg::rosidl_typesupport_introspection_cpp::MotorSpeed_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, controller, msg, MotorSpeed)() {
  return &::controller::msg::rosidl_typesupport_introspection_cpp::MotorSpeed_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
