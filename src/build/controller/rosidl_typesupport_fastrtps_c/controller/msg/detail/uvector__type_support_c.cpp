// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from controller:msg/Uvector.idl
// generated code does not contain a copyright notice
#include "controller/msg/detail/uvector__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "controller/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "controller/msg/detail/uvector__struct.h"
#include "controller/msg/detail/uvector__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _Uvector__ros_msg_type = controller__msg__Uvector;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_controller
bool cdr_serialize_controller__msg__Uvector(
  const controller__msg__Uvector * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: value
  {
    size_t size = 12;
    auto array_ptr = ros_message->value;
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_controller
bool cdr_deserialize_controller__msg__Uvector(
  eprosima::fastcdr::Cdr & cdr,
  controller__msg__Uvector * ros_message)
{
  // Field name: value
  {
    size_t size = 12;
    auto array_ptr = ros_message->value;
    cdr.deserialize_array(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_controller
size_t get_serialized_size_controller__msg__Uvector(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Uvector__ros_msg_type * ros_message = static_cast<const _Uvector__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: value
  {
    size_t array_size = 12;
    auto array_ptr = ros_message->value;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_controller
size_t max_serialized_size_controller__msg__Uvector(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: value
  {
    size_t array_size = 12;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = controller__msg__Uvector;
    is_plain =
      (
      offsetof(DataType, value) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_controller
bool cdr_serialize_key_controller__msg__Uvector(
  const controller__msg__Uvector * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: value
  {
    size_t size = 12;
    auto array_ptr = ros_message->value;
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_controller
size_t get_serialized_size_key_controller__msg__Uvector(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Uvector__ros_msg_type * ros_message = static_cast<const _Uvector__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: value
  {
    size_t array_size = 12;
    auto array_ptr = ros_message->value;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_controller
size_t max_serialized_size_key_controller__msg__Uvector(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: value
  {
    size_t array_size = 12;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = controller__msg__Uvector;
    is_plain =
      (
      offsetof(DataType, value) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _Uvector__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const controller__msg__Uvector * ros_message = static_cast<const controller__msg__Uvector *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_controller__msg__Uvector(ros_message, cdr);
}

static bool _Uvector__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  controller__msg__Uvector * ros_message = static_cast<controller__msg__Uvector *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_controller__msg__Uvector(cdr, ros_message);
}

static uint32_t _Uvector__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_controller__msg__Uvector(
      untyped_ros_message, 0));
}

static size_t _Uvector__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_controller__msg__Uvector(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Uvector = {
  "controller::msg",
  "Uvector",
  _Uvector__cdr_serialize,
  _Uvector__cdr_deserialize,
  _Uvector__get_serialized_size,
  _Uvector__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _Uvector__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Uvector,
  get_message_typesupport_handle_function,
  &controller__msg__Uvector__get_type_hash,
  &controller__msg__Uvector__get_type_description,
  &controller__msg__Uvector__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, controller, msg, Uvector)() {
  return &_Uvector__type_support;
}

#if defined(__cplusplus)
}
#endif
