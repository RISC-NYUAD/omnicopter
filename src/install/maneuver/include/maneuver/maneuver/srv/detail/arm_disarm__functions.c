// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from maneuver:srv/ArmDisarm.idl
// generated code does not contain a copyright notice
#include "maneuver/srv/detail/arm_disarm__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
maneuver__srv__ArmDisarm_Request__init(maneuver__srv__ArmDisarm_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
maneuver__srv__ArmDisarm_Request__fini(maneuver__srv__ArmDisarm_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
maneuver__srv__ArmDisarm_Request__are_equal(const maneuver__srv__ArmDisarm_Request * lhs, const maneuver__srv__ArmDisarm_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
maneuver__srv__ArmDisarm_Request__copy(
  const maneuver__srv__ArmDisarm_Request * input,
  maneuver__srv__ArmDisarm_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

maneuver__srv__ArmDisarm_Request *
maneuver__srv__ArmDisarm_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  maneuver__srv__ArmDisarm_Request * msg = (maneuver__srv__ArmDisarm_Request *)allocator.allocate(sizeof(maneuver__srv__ArmDisarm_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(maneuver__srv__ArmDisarm_Request));
  bool success = maneuver__srv__ArmDisarm_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
maneuver__srv__ArmDisarm_Request__destroy(maneuver__srv__ArmDisarm_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    maneuver__srv__ArmDisarm_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
maneuver__srv__ArmDisarm_Request__Sequence__init(maneuver__srv__ArmDisarm_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  maneuver__srv__ArmDisarm_Request * data = NULL;

  if (size) {
    data = (maneuver__srv__ArmDisarm_Request *)allocator.zero_allocate(size, sizeof(maneuver__srv__ArmDisarm_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = maneuver__srv__ArmDisarm_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        maneuver__srv__ArmDisarm_Request__fini(&data[i - 1]);
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
maneuver__srv__ArmDisarm_Request__Sequence__fini(maneuver__srv__ArmDisarm_Request__Sequence * array)
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
      maneuver__srv__ArmDisarm_Request__fini(&array->data[i]);
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

maneuver__srv__ArmDisarm_Request__Sequence *
maneuver__srv__ArmDisarm_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  maneuver__srv__ArmDisarm_Request__Sequence * array = (maneuver__srv__ArmDisarm_Request__Sequence *)allocator.allocate(sizeof(maneuver__srv__ArmDisarm_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = maneuver__srv__ArmDisarm_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
maneuver__srv__ArmDisarm_Request__Sequence__destroy(maneuver__srv__ArmDisarm_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    maneuver__srv__ArmDisarm_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
maneuver__srv__ArmDisarm_Request__Sequence__are_equal(const maneuver__srv__ArmDisarm_Request__Sequence * lhs, const maneuver__srv__ArmDisarm_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!maneuver__srv__ArmDisarm_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
maneuver__srv__ArmDisarm_Request__Sequence__copy(
  const maneuver__srv__ArmDisarm_Request__Sequence * input,
  maneuver__srv__ArmDisarm_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(maneuver__srv__ArmDisarm_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    maneuver__srv__ArmDisarm_Request * data =
      (maneuver__srv__ArmDisarm_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!maneuver__srv__ArmDisarm_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          maneuver__srv__ArmDisarm_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!maneuver__srv__ArmDisarm_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
maneuver__srv__ArmDisarm_Response__init(maneuver__srv__ArmDisarm_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
maneuver__srv__ArmDisarm_Response__fini(maneuver__srv__ArmDisarm_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
maneuver__srv__ArmDisarm_Response__are_equal(const maneuver__srv__ArmDisarm_Response * lhs, const maneuver__srv__ArmDisarm_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
maneuver__srv__ArmDisarm_Response__copy(
  const maneuver__srv__ArmDisarm_Response * input,
  maneuver__srv__ArmDisarm_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

maneuver__srv__ArmDisarm_Response *
maneuver__srv__ArmDisarm_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  maneuver__srv__ArmDisarm_Response * msg = (maneuver__srv__ArmDisarm_Response *)allocator.allocate(sizeof(maneuver__srv__ArmDisarm_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(maneuver__srv__ArmDisarm_Response));
  bool success = maneuver__srv__ArmDisarm_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
maneuver__srv__ArmDisarm_Response__destroy(maneuver__srv__ArmDisarm_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    maneuver__srv__ArmDisarm_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
maneuver__srv__ArmDisarm_Response__Sequence__init(maneuver__srv__ArmDisarm_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  maneuver__srv__ArmDisarm_Response * data = NULL;

  if (size) {
    data = (maneuver__srv__ArmDisarm_Response *)allocator.zero_allocate(size, sizeof(maneuver__srv__ArmDisarm_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = maneuver__srv__ArmDisarm_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        maneuver__srv__ArmDisarm_Response__fini(&data[i - 1]);
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
maneuver__srv__ArmDisarm_Response__Sequence__fini(maneuver__srv__ArmDisarm_Response__Sequence * array)
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
      maneuver__srv__ArmDisarm_Response__fini(&array->data[i]);
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

maneuver__srv__ArmDisarm_Response__Sequence *
maneuver__srv__ArmDisarm_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  maneuver__srv__ArmDisarm_Response__Sequence * array = (maneuver__srv__ArmDisarm_Response__Sequence *)allocator.allocate(sizeof(maneuver__srv__ArmDisarm_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = maneuver__srv__ArmDisarm_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
maneuver__srv__ArmDisarm_Response__Sequence__destroy(maneuver__srv__ArmDisarm_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    maneuver__srv__ArmDisarm_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
maneuver__srv__ArmDisarm_Response__Sequence__are_equal(const maneuver__srv__ArmDisarm_Response__Sequence * lhs, const maneuver__srv__ArmDisarm_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!maneuver__srv__ArmDisarm_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
maneuver__srv__ArmDisarm_Response__Sequence__copy(
  const maneuver__srv__ArmDisarm_Response__Sequence * input,
  maneuver__srv__ArmDisarm_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(maneuver__srv__ArmDisarm_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    maneuver__srv__ArmDisarm_Response * data =
      (maneuver__srv__ArmDisarm_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!maneuver__srv__ArmDisarm_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          maneuver__srv__ArmDisarm_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!maneuver__srv__ArmDisarm_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "maneuver/srv/detail/arm_disarm__functions.h"

bool
maneuver__srv__ArmDisarm_Event__init(maneuver__srv__ArmDisarm_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    maneuver__srv__ArmDisarm_Event__fini(msg);
    return false;
  }
  // request
  if (!maneuver__srv__ArmDisarm_Request__Sequence__init(&msg->request, 0)) {
    maneuver__srv__ArmDisarm_Event__fini(msg);
    return false;
  }
  // response
  if (!maneuver__srv__ArmDisarm_Response__Sequence__init(&msg->response, 0)) {
    maneuver__srv__ArmDisarm_Event__fini(msg);
    return false;
  }
  return true;
}

void
maneuver__srv__ArmDisarm_Event__fini(maneuver__srv__ArmDisarm_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  maneuver__srv__ArmDisarm_Request__Sequence__fini(&msg->request);
  // response
  maneuver__srv__ArmDisarm_Response__Sequence__fini(&msg->response);
}

bool
maneuver__srv__ArmDisarm_Event__are_equal(const maneuver__srv__ArmDisarm_Event * lhs, const maneuver__srv__ArmDisarm_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!maneuver__srv__ArmDisarm_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!maneuver__srv__ArmDisarm_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
maneuver__srv__ArmDisarm_Event__copy(
  const maneuver__srv__ArmDisarm_Event * input,
  maneuver__srv__ArmDisarm_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!maneuver__srv__ArmDisarm_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!maneuver__srv__ArmDisarm_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

maneuver__srv__ArmDisarm_Event *
maneuver__srv__ArmDisarm_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  maneuver__srv__ArmDisarm_Event * msg = (maneuver__srv__ArmDisarm_Event *)allocator.allocate(sizeof(maneuver__srv__ArmDisarm_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(maneuver__srv__ArmDisarm_Event));
  bool success = maneuver__srv__ArmDisarm_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
maneuver__srv__ArmDisarm_Event__destroy(maneuver__srv__ArmDisarm_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    maneuver__srv__ArmDisarm_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
maneuver__srv__ArmDisarm_Event__Sequence__init(maneuver__srv__ArmDisarm_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  maneuver__srv__ArmDisarm_Event * data = NULL;

  if (size) {
    data = (maneuver__srv__ArmDisarm_Event *)allocator.zero_allocate(size, sizeof(maneuver__srv__ArmDisarm_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = maneuver__srv__ArmDisarm_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        maneuver__srv__ArmDisarm_Event__fini(&data[i - 1]);
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
maneuver__srv__ArmDisarm_Event__Sequence__fini(maneuver__srv__ArmDisarm_Event__Sequence * array)
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
      maneuver__srv__ArmDisarm_Event__fini(&array->data[i]);
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

maneuver__srv__ArmDisarm_Event__Sequence *
maneuver__srv__ArmDisarm_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  maneuver__srv__ArmDisarm_Event__Sequence * array = (maneuver__srv__ArmDisarm_Event__Sequence *)allocator.allocate(sizeof(maneuver__srv__ArmDisarm_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = maneuver__srv__ArmDisarm_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
maneuver__srv__ArmDisarm_Event__Sequence__destroy(maneuver__srv__ArmDisarm_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    maneuver__srv__ArmDisarm_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
maneuver__srv__ArmDisarm_Event__Sequence__are_equal(const maneuver__srv__ArmDisarm_Event__Sequence * lhs, const maneuver__srv__ArmDisarm_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!maneuver__srv__ArmDisarm_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
maneuver__srv__ArmDisarm_Event__Sequence__copy(
  const maneuver__srv__ArmDisarm_Event__Sequence * input,
  maneuver__srv__ArmDisarm_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(maneuver__srv__ArmDisarm_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    maneuver__srv__ArmDisarm_Event * data =
      (maneuver__srv__ArmDisarm_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!maneuver__srv__ArmDisarm_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          maneuver__srv__ArmDisarm_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!maneuver__srv__ArmDisarm_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
