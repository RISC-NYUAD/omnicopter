// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from controller:msg/ErrorMsg.idl
// generated code does not contain a copyright notice
#include "controller/msg/detail/error_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `ex`
// Member `ev`
// Member `ea`
// Member `er`
// Member `ew`
// Member `iex`
// Member `ier`
// Member `accd`
#include "geometry_msgs/msg/detail/vector3__functions.h"
// Member `wrench`
// Member `exwrench`
// Member `prop`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
controller__msg__ErrorMsg__init(controller__msg__ErrorMsg * msg)
{
  if (!msg) {
    return false;
  }
  // ex
  if (!geometry_msgs__msg__Vector3__init(&msg->ex)) {
    controller__msg__ErrorMsg__fini(msg);
    return false;
  }
  // ev
  if (!geometry_msgs__msg__Vector3__init(&msg->ev)) {
    controller__msg__ErrorMsg__fini(msg);
    return false;
  }
  // ea
  if (!geometry_msgs__msg__Vector3__init(&msg->ea)) {
    controller__msg__ErrorMsg__fini(msg);
    return false;
  }
  // er
  if (!geometry_msgs__msg__Vector3__init(&msg->er)) {
    controller__msg__ErrorMsg__fini(msg);
    return false;
  }
  // ew
  if (!geometry_msgs__msg__Vector3__init(&msg->ew)) {
    controller__msg__ErrorMsg__fini(msg);
    return false;
  }
  // iex
  if (!geometry_msgs__msg__Vector3__init(&msg->iex)) {
    controller__msg__ErrorMsg__fini(msg);
    return false;
  }
  // ier
  if (!geometry_msgs__msg__Vector3__init(&msg->ier)) {
    controller__msg__ErrorMsg__fini(msg);
    return false;
  }
  // accd
  if (!geometry_msgs__msg__Vector3__init(&msg->accd)) {
    controller__msg__ErrorMsg__fini(msg);
    return false;
  }
  // wrench
  if (!rosidl_runtime_c__float__Sequence__init(&msg->wrench, 0)) {
    controller__msg__ErrorMsg__fini(msg);
    return false;
  }
  // exwrench
  if (!rosidl_runtime_c__float__Sequence__init(&msg->exwrench, 0)) {
    controller__msg__ErrorMsg__fini(msg);
    return false;
  }
  // prop
  if (!rosidl_runtime_c__float__Sequence__init(&msg->prop, 0)) {
    controller__msg__ErrorMsg__fini(msg);
    return false;
  }
  // weight
  return true;
}

void
controller__msg__ErrorMsg__fini(controller__msg__ErrorMsg * msg)
{
  if (!msg) {
    return;
  }
  // ex
  geometry_msgs__msg__Vector3__fini(&msg->ex);
  // ev
  geometry_msgs__msg__Vector3__fini(&msg->ev);
  // ea
  geometry_msgs__msg__Vector3__fini(&msg->ea);
  // er
  geometry_msgs__msg__Vector3__fini(&msg->er);
  // ew
  geometry_msgs__msg__Vector3__fini(&msg->ew);
  // iex
  geometry_msgs__msg__Vector3__fini(&msg->iex);
  // ier
  geometry_msgs__msg__Vector3__fini(&msg->ier);
  // accd
  geometry_msgs__msg__Vector3__fini(&msg->accd);
  // wrench
  rosidl_runtime_c__float__Sequence__fini(&msg->wrench);
  // exwrench
  rosidl_runtime_c__float__Sequence__fini(&msg->exwrench);
  // prop
  rosidl_runtime_c__float__Sequence__fini(&msg->prop);
  // weight
}

bool
controller__msg__ErrorMsg__are_equal(const controller__msg__ErrorMsg * lhs, const controller__msg__ErrorMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // ex
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->ex), &(rhs->ex)))
  {
    return false;
  }
  // ev
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->ev), &(rhs->ev)))
  {
    return false;
  }
  // ea
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->ea), &(rhs->ea)))
  {
    return false;
  }
  // er
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->er), &(rhs->er)))
  {
    return false;
  }
  // ew
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->ew), &(rhs->ew)))
  {
    return false;
  }
  // iex
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->iex), &(rhs->iex)))
  {
    return false;
  }
  // ier
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->ier), &(rhs->ier)))
  {
    return false;
  }
  // accd
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->accd), &(rhs->accd)))
  {
    return false;
  }
  // wrench
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->wrench), &(rhs->wrench)))
  {
    return false;
  }
  // exwrench
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->exwrench), &(rhs->exwrench)))
  {
    return false;
  }
  // prop
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->prop), &(rhs->prop)))
  {
    return false;
  }
  // weight
  if (lhs->weight != rhs->weight) {
    return false;
  }
  return true;
}

bool
controller__msg__ErrorMsg__copy(
  const controller__msg__ErrorMsg * input,
  controller__msg__ErrorMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // ex
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->ex), &(output->ex)))
  {
    return false;
  }
  // ev
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->ev), &(output->ev)))
  {
    return false;
  }
  // ea
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->ea), &(output->ea)))
  {
    return false;
  }
  // er
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->er), &(output->er)))
  {
    return false;
  }
  // ew
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->ew), &(output->ew)))
  {
    return false;
  }
  // iex
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->iex), &(output->iex)))
  {
    return false;
  }
  // ier
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->ier), &(output->ier)))
  {
    return false;
  }
  // accd
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->accd), &(output->accd)))
  {
    return false;
  }
  // wrench
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->wrench), &(output->wrench)))
  {
    return false;
  }
  // exwrench
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->exwrench), &(output->exwrench)))
  {
    return false;
  }
  // prop
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->prop), &(output->prop)))
  {
    return false;
  }
  // weight
  output->weight = input->weight;
  return true;
}

controller__msg__ErrorMsg *
controller__msg__ErrorMsg__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__ErrorMsg * msg = (controller__msg__ErrorMsg *)allocator.allocate(sizeof(controller__msg__ErrorMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(controller__msg__ErrorMsg));
  bool success = controller__msg__ErrorMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
controller__msg__ErrorMsg__destroy(controller__msg__ErrorMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    controller__msg__ErrorMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
controller__msg__ErrorMsg__Sequence__init(controller__msg__ErrorMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__ErrorMsg * data = NULL;

  if (size) {
    data = (controller__msg__ErrorMsg *)allocator.zero_allocate(size, sizeof(controller__msg__ErrorMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = controller__msg__ErrorMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        controller__msg__ErrorMsg__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
controller__msg__ErrorMsg__Sequence__fini(controller__msg__ErrorMsg__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      controller__msg__ErrorMsg__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

controller__msg__ErrorMsg__Sequence *
controller__msg__ErrorMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__ErrorMsg__Sequence * array = (controller__msg__ErrorMsg__Sequence *)allocator.allocate(sizeof(controller__msg__ErrorMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = controller__msg__ErrorMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
controller__msg__ErrorMsg__Sequence__destroy(controller__msg__ErrorMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    controller__msg__ErrorMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
controller__msg__ErrorMsg__Sequence__are_equal(const controller__msg__ErrorMsg__Sequence * lhs, const controller__msg__ErrorMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!controller__msg__ErrorMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
controller__msg__ErrorMsg__Sequence__copy(
  const controller__msg__ErrorMsg__Sequence * input,
  controller__msg__ErrorMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(controller__msg__ErrorMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    controller__msg__ErrorMsg * data =
      (controller__msg__ErrorMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!controller__msg__ErrorMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          controller__msg__ErrorMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!controller__msg__ErrorMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
