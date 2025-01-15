// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from maneuver:msg/FullPose.idl
// generated code does not contain a copyright notice
#include "maneuver/msg/detail/full_pose__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `vel`
#include "geometry_msgs/msg/detail/twist__functions.h"
// Member `acc`
#include "geometry_msgs/msg/detail/accel__functions.h"

bool
maneuver__msg__FullPose__init(maneuver__msg__FullPose * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    maneuver__msg__FullPose__fini(msg);
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    maneuver__msg__FullPose__fini(msg);
    return false;
  }
  // vel
  if (!geometry_msgs__msg__Twist__init(&msg->vel)) {
    maneuver__msg__FullPose__fini(msg);
    return false;
  }
  // acc
  if (!geometry_msgs__msg__Accel__init(&msg->acc)) {
    maneuver__msg__FullPose__fini(msg);
    return false;
  }
  return true;
}

void
maneuver__msg__FullPose__fini(maneuver__msg__FullPose * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
  // vel
  geometry_msgs__msg__Twist__fini(&msg->vel);
  // acc
  geometry_msgs__msg__Accel__fini(&msg->acc);
}

bool
maneuver__msg__FullPose__are_equal(const maneuver__msg__FullPose * lhs, const maneuver__msg__FullPose * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // vel
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->vel), &(rhs->vel)))
  {
    return false;
  }
  // acc
  if (!geometry_msgs__msg__Accel__are_equal(
      &(lhs->acc), &(rhs->acc)))
  {
    return false;
  }
  return true;
}

bool
maneuver__msg__FullPose__copy(
  const maneuver__msg__FullPose * input,
  maneuver__msg__FullPose * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // vel
  if (!geometry_msgs__msg__Twist__copy(
      &(input->vel), &(output->vel)))
  {
    return false;
  }
  // acc
  if (!geometry_msgs__msg__Accel__copy(
      &(input->acc), &(output->acc)))
  {
    return false;
  }
  return true;
}

maneuver__msg__FullPose *
maneuver__msg__FullPose__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  maneuver__msg__FullPose * msg = (maneuver__msg__FullPose *)allocator.allocate(sizeof(maneuver__msg__FullPose), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(maneuver__msg__FullPose));
  bool success = maneuver__msg__FullPose__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
maneuver__msg__FullPose__destroy(maneuver__msg__FullPose * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    maneuver__msg__FullPose__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
maneuver__msg__FullPose__Sequence__init(maneuver__msg__FullPose__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  maneuver__msg__FullPose * data = NULL;

  if (size) {
    data = (maneuver__msg__FullPose *)allocator.zero_allocate(size, sizeof(maneuver__msg__FullPose), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = maneuver__msg__FullPose__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        maneuver__msg__FullPose__fini(&data[i - 1]);
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
maneuver__msg__FullPose__Sequence__fini(maneuver__msg__FullPose__Sequence * array)
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
      maneuver__msg__FullPose__fini(&array->data[i]);
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

maneuver__msg__FullPose__Sequence *
maneuver__msg__FullPose__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  maneuver__msg__FullPose__Sequence * array = (maneuver__msg__FullPose__Sequence *)allocator.allocate(sizeof(maneuver__msg__FullPose__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = maneuver__msg__FullPose__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
maneuver__msg__FullPose__Sequence__destroy(maneuver__msg__FullPose__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    maneuver__msg__FullPose__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
maneuver__msg__FullPose__Sequence__are_equal(const maneuver__msg__FullPose__Sequence * lhs, const maneuver__msg__FullPose__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!maneuver__msg__FullPose__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
maneuver__msg__FullPose__Sequence__copy(
  const maneuver__msg__FullPose__Sequence * input,
  maneuver__msg__FullPose__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(maneuver__msg__FullPose);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    maneuver__msg__FullPose * data =
      (maneuver__msg__FullPose *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!maneuver__msg__FullPose__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          maneuver__msg__FullPose__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!maneuver__msg__FullPose__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
