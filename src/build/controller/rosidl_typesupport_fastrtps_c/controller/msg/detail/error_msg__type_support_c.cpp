// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from controller:msg/ErrorMsg.idl
// generated code does not contain a copyright notice
#include "controller/msg/detail/error_msg__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "controller/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "controller/msg/detail/error_msg__struct.h"
#include "controller/msg/detail/error_msg__functions.h"
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

#include "geometry_msgs/msg/detail/vector3__functions.h"  // accd, ea, er, ev, ew, ex, ier, iex
#include "rosidl_runtime_c/primitives_sequence.h"  // exwrench, prop, wrench
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // exwrench, prop, wrench

// forward declare type support functions

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_controller
bool cdr_serialize_geometry_msgs__msg__Vector3(
  const geometry_msgs__msg__Vector3 * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_controller
bool cdr_deserialize_geometry_msgs__msg__Vector3(
  eprosima::fastcdr::Cdr & cdr,
  geometry_msgs__msg__Vector3 * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_controller
size_t get_serialized_size_geometry_msgs__msg__Vector3(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_controller
size_t max_serialized_size_geometry_msgs__msg__Vector3(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_controller
bool cdr_serialize_key_geometry_msgs__msg__Vector3(
  const geometry_msgs__msg__Vector3 * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_controller
size_t get_serialized_size_key_geometry_msgs__msg__Vector3(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_controller
size_t max_serialized_size_key_geometry_msgs__msg__Vector3(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_controller
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Vector3)();


using _ErrorMsg__ros_msg_type = controller__msg__ErrorMsg;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_controller
bool cdr_serialize_controller__msg__ErrorMsg(
  const controller__msg__ErrorMsg * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: ex
  {
    cdr_serialize_geometry_msgs__msg__Vector3(
      &ros_message->ex, cdr);
  }

  // Field name: ev
  {
    cdr_serialize_geometry_msgs__msg__Vector3(
      &ros_message->ev, cdr);
  }

  // Field name: ea
  {
    cdr_serialize_geometry_msgs__msg__Vector3(
      &ros_message->ea, cdr);
  }

  // Field name: er
  {
    cdr_serialize_geometry_msgs__msg__Vector3(
      &ros_message->er, cdr);
  }

  // Field name: ew
  {
    cdr_serialize_geometry_msgs__msg__Vector3(
      &ros_message->ew, cdr);
  }

  // Field name: iex
  {
    cdr_serialize_geometry_msgs__msg__Vector3(
      &ros_message->iex, cdr);
  }

  // Field name: ier
  {
    cdr_serialize_geometry_msgs__msg__Vector3(
      &ros_message->ier, cdr);
  }

  // Field name: accd
  {
    cdr_serialize_geometry_msgs__msg__Vector3(
      &ros_message->accd, cdr);
  }

  // Field name: wrench
  {
    size_t size = ros_message->wrench.size;
    auto array_ptr = ros_message->wrench.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: exwrench
  {
    size_t size = ros_message->exwrench.size;
    auto array_ptr = ros_message->exwrench.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: prop
  {
    size_t size = ros_message->prop.size;
    auto array_ptr = ros_message->prop.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: weight
  {
    cdr << ros_message->weight;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_controller
bool cdr_deserialize_controller__msg__ErrorMsg(
  eprosima::fastcdr::Cdr & cdr,
  controller__msg__ErrorMsg * ros_message)
{
  // Field name: ex
  {
    cdr_deserialize_geometry_msgs__msg__Vector3(cdr, &ros_message->ex);
  }

  // Field name: ev
  {
    cdr_deserialize_geometry_msgs__msg__Vector3(cdr, &ros_message->ev);
  }

  // Field name: ea
  {
    cdr_deserialize_geometry_msgs__msg__Vector3(cdr, &ros_message->ea);
  }

  // Field name: er
  {
    cdr_deserialize_geometry_msgs__msg__Vector3(cdr, &ros_message->er);
  }

  // Field name: ew
  {
    cdr_deserialize_geometry_msgs__msg__Vector3(cdr, &ros_message->ew);
  }

  // Field name: iex
  {
    cdr_deserialize_geometry_msgs__msg__Vector3(cdr, &ros_message->iex);
  }

  // Field name: ier
  {
    cdr_deserialize_geometry_msgs__msg__Vector3(cdr, &ros_message->ier);
  }

  // Field name: accd
  {
    cdr_deserialize_geometry_msgs__msg__Vector3(cdr, &ros_message->accd);
  }

  // Field name: wrench
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->wrench.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->wrench);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->wrench, size)) {
      fprintf(stderr, "failed to create array for field 'wrench'");
      return false;
    }
    auto array_ptr = ros_message->wrench.data;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: exwrench
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->exwrench.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->exwrench);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->exwrench, size)) {
      fprintf(stderr, "failed to create array for field 'exwrench'");
      return false;
    }
    auto array_ptr = ros_message->exwrench.data;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: prop
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->prop.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->prop);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->prop, size)) {
      fprintf(stderr, "failed to create array for field 'prop'");
      return false;
    }
    auto array_ptr = ros_message->prop.data;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: weight
  {
    cdr >> ros_message->weight;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_controller
size_t get_serialized_size_controller__msg__ErrorMsg(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ErrorMsg__ros_msg_type * ros_message = static_cast<const _ErrorMsg__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: ex
  current_alignment += get_serialized_size_geometry_msgs__msg__Vector3(
    &(ros_message->ex), current_alignment);

  // Field name: ev
  current_alignment += get_serialized_size_geometry_msgs__msg__Vector3(
    &(ros_message->ev), current_alignment);

  // Field name: ea
  current_alignment += get_serialized_size_geometry_msgs__msg__Vector3(
    &(ros_message->ea), current_alignment);

  // Field name: er
  current_alignment += get_serialized_size_geometry_msgs__msg__Vector3(
    &(ros_message->er), current_alignment);

  // Field name: ew
  current_alignment += get_serialized_size_geometry_msgs__msg__Vector3(
    &(ros_message->ew), current_alignment);

  // Field name: iex
  current_alignment += get_serialized_size_geometry_msgs__msg__Vector3(
    &(ros_message->iex), current_alignment);

  // Field name: ier
  current_alignment += get_serialized_size_geometry_msgs__msg__Vector3(
    &(ros_message->ier), current_alignment);

  // Field name: accd
  current_alignment += get_serialized_size_geometry_msgs__msg__Vector3(
    &(ros_message->accd), current_alignment);

  // Field name: wrench
  {
    size_t array_size = ros_message->wrench.size;
    auto array_ptr = ros_message->wrench.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: exwrench
  {
    size_t array_size = ros_message->exwrench.size;
    auto array_ptr = ros_message->exwrench.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: prop
  {
    size_t array_size = ros_message->prop.size;
    auto array_ptr = ros_message->prop.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: weight
  {
    size_t item_size = sizeof(ros_message->weight);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_controller
size_t max_serialized_size_controller__msg__ErrorMsg(
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

  // Field name: ex
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: ev
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: ea
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: er
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: ew
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: iex
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: ier
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: accd
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: wrench
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: exwrench
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: prop
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: weight
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = controller__msg__ErrorMsg;
    is_plain =
      (
      offsetof(DataType, weight) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_controller
bool cdr_serialize_key_controller__msg__ErrorMsg(
  const controller__msg__ErrorMsg * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: ex
  {
    cdr_serialize_key_geometry_msgs__msg__Vector3(
      &ros_message->ex, cdr);
  }

  // Field name: ev
  {
    cdr_serialize_key_geometry_msgs__msg__Vector3(
      &ros_message->ev, cdr);
  }

  // Field name: ea
  {
    cdr_serialize_key_geometry_msgs__msg__Vector3(
      &ros_message->ea, cdr);
  }

  // Field name: er
  {
    cdr_serialize_key_geometry_msgs__msg__Vector3(
      &ros_message->er, cdr);
  }

  // Field name: ew
  {
    cdr_serialize_key_geometry_msgs__msg__Vector3(
      &ros_message->ew, cdr);
  }

  // Field name: iex
  {
    cdr_serialize_key_geometry_msgs__msg__Vector3(
      &ros_message->iex, cdr);
  }

  // Field name: ier
  {
    cdr_serialize_key_geometry_msgs__msg__Vector3(
      &ros_message->ier, cdr);
  }

  // Field name: accd
  {
    cdr_serialize_key_geometry_msgs__msg__Vector3(
      &ros_message->accd, cdr);
  }

  // Field name: wrench
  {
    size_t size = ros_message->wrench.size;
    auto array_ptr = ros_message->wrench.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: exwrench
  {
    size_t size = ros_message->exwrench.size;
    auto array_ptr = ros_message->exwrench.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: prop
  {
    size_t size = ros_message->prop.size;
    auto array_ptr = ros_message->prop.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: weight
  {
    cdr << ros_message->weight;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_controller
size_t get_serialized_size_key_controller__msg__ErrorMsg(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ErrorMsg__ros_msg_type * ros_message = static_cast<const _ErrorMsg__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: ex
  current_alignment += get_serialized_size_key_geometry_msgs__msg__Vector3(
    &(ros_message->ex), current_alignment);

  // Field name: ev
  current_alignment += get_serialized_size_key_geometry_msgs__msg__Vector3(
    &(ros_message->ev), current_alignment);

  // Field name: ea
  current_alignment += get_serialized_size_key_geometry_msgs__msg__Vector3(
    &(ros_message->ea), current_alignment);

  // Field name: er
  current_alignment += get_serialized_size_key_geometry_msgs__msg__Vector3(
    &(ros_message->er), current_alignment);

  // Field name: ew
  current_alignment += get_serialized_size_key_geometry_msgs__msg__Vector3(
    &(ros_message->ew), current_alignment);

  // Field name: iex
  current_alignment += get_serialized_size_key_geometry_msgs__msg__Vector3(
    &(ros_message->iex), current_alignment);

  // Field name: ier
  current_alignment += get_serialized_size_key_geometry_msgs__msg__Vector3(
    &(ros_message->ier), current_alignment);

  // Field name: accd
  current_alignment += get_serialized_size_key_geometry_msgs__msg__Vector3(
    &(ros_message->accd), current_alignment);

  // Field name: wrench
  {
    size_t array_size = ros_message->wrench.size;
    auto array_ptr = ros_message->wrench.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: exwrench
  {
    size_t array_size = ros_message->exwrench.size;
    auto array_ptr = ros_message->exwrench.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: prop
  {
    size_t array_size = ros_message->prop.size;
    auto array_ptr = ros_message->prop.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: weight
  {
    size_t item_size = sizeof(ros_message->weight);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_controller
size_t max_serialized_size_key_controller__msg__ErrorMsg(
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
  // Field name: ex
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: ev
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: ea
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: er
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: ew
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: iex
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: ier
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: accd
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: wrench
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: exwrench
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: prop
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: weight
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = controller__msg__ErrorMsg;
    is_plain =
      (
      offsetof(DataType, weight) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _ErrorMsg__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const controller__msg__ErrorMsg * ros_message = static_cast<const controller__msg__ErrorMsg *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_controller__msg__ErrorMsg(ros_message, cdr);
}

static bool _ErrorMsg__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  controller__msg__ErrorMsg * ros_message = static_cast<controller__msg__ErrorMsg *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_controller__msg__ErrorMsg(cdr, ros_message);
}

static uint32_t _ErrorMsg__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_controller__msg__ErrorMsg(
      untyped_ros_message, 0));
}

static size_t _ErrorMsg__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_controller__msg__ErrorMsg(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ErrorMsg = {
  "controller::msg",
  "ErrorMsg",
  _ErrorMsg__cdr_serialize,
  _ErrorMsg__cdr_deserialize,
  _ErrorMsg__get_serialized_size,
  _ErrorMsg__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ErrorMsg__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ErrorMsg,
  get_message_typesupport_handle_function,
  &controller__msg__ErrorMsg__get_type_hash,
  &controller__msg__ErrorMsg__get_type_description,
  &controller__msg__ErrorMsg__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, controller, msg, ErrorMsg)() {
  return &_ErrorMsg__type_support;
}

#if defined(__cplusplus)
}
#endif
