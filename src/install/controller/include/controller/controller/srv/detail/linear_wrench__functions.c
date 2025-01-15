// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from controller:srv/LinearWrench.idl
// generated code does not contain a copyright notice
#include "controller/srv/detail/linear_wrench__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
controller__srv__LinearWrench_Request__init(controller__srv__LinearWrench_Request * msg)
{
  if (!msg) {
    return false;
  }
  // fx1
  // fy1
  // fz1
  // fx2
  // fy2
  // fz2
  // ramp
  // duration
  return true;
}

void
controller__srv__LinearWrench_Request__fini(controller__srv__LinearWrench_Request * msg)
{
  if (!msg) {
    return;
  }
  // fx1
  // fy1
  // fz1
  // fx2
  // fy2
  // fz2
  // ramp
  // duration
}

bool
controller__srv__LinearWrench_Request__are_equal(const controller__srv__LinearWrench_Request * lhs, const controller__srv__LinearWrench_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // fx1
  if (lhs->fx1 != rhs->fx1) {
    return false;
  }
  // fy1
  if (lhs->fy1 != rhs->fy1) {
    return false;
  }
  // fz1
  if (lhs->fz1 != rhs->fz1) {
    return false;
  }
  // fx2
  if (lhs->fx2 != rhs->fx2) {
    return false;
  }
  // fy2
  if (lhs->fy2 != rhs->fy2) {
    return false;
  }
  // fz2
  if (lhs->fz2 != rhs->fz2) {
    return false;
  }
  // ramp
  if (lhs->ramp != rhs->ramp) {
    return false;
  }
  // duration
  if (lhs->duration != rhs->duration) {
    return false;
  }
  return true;
}

bool
controller__srv__LinearWrench_Request__copy(
  const controller__srv__LinearWrench_Request * input,
  controller__srv__LinearWrench_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // fx1
  output->fx1 = input->fx1;
  // fy1
  output->fy1 = input->fy1;
  // fz1
  output->fz1 = input->fz1;
  // fx2
  output->fx2 = input->fx2;
  // fy2
  output->fy2 = input->fy2;
  // fz2
  output->fz2 = input->fz2;
  // ramp
  output->ramp = input->ramp;
  // duration
  output->duration = input->duration;
  return true;
}

controller__srv__LinearWrench_Request *
controller__srv__LinearWrench_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__srv__LinearWrench_Request * msg = (controller__srv__LinearWrench_Request *)allocator.allocate(sizeof(controller__srv__LinearWrench_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(controller__srv__LinearWrench_Request));
  bool success = controller__srv__LinearWrench_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
controller__srv__LinearWrench_Request__destroy(controller__srv__LinearWrench_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    controller__srv__LinearWrench_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
controller__srv__LinearWrench_Request__Sequence__init(controller__srv__LinearWrench_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__srv__LinearWrench_Request * data = NULL;

  if (size) {
    data = (controller__srv__LinearWrench_Request *)allocator.zero_allocate(size, sizeof(controller__srv__LinearWrench_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = controller__srv__LinearWrench_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        controller__srv__LinearWrench_Request__fini(&data[i - 1]);
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
controller__srv__LinearWrench_Request__Sequence__fini(controller__srv__LinearWrench_Request__Sequence * array)
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
      controller__srv__LinearWrench_Request__fini(&array->data[i]);
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

controller__srv__LinearWrench_Request__Sequence *
controller__srv__LinearWrench_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__srv__LinearWrench_Request__Sequence * array = (controller__srv__LinearWrench_Request__Sequence *)allocator.allocate(sizeof(controller__srv__LinearWrench_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = controller__srv__LinearWrench_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
controller__srv__LinearWrench_Request__Sequence__destroy(controller__srv__LinearWrench_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    controller__srv__LinearWrench_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
controller__srv__LinearWrench_Request__Sequence__are_equal(const controller__srv__LinearWrench_Request__Sequence * lhs, const controller__srv__LinearWrench_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!controller__srv__LinearWrench_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
controller__srv__LinearWrench_Request__Sequence__copy(
  const controller__srv__LinearWrench_Request__Sequence * input,
  controller__srv__LinearWrench_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(controller__srv__LinearWrench_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    controller__srv__LinearWrench_Request * data =
      (controller__srv__LinearWrench_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!controller__srv__LinearWrench_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          controller__srv__LinearWrench_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!controller__srv__LinearWrench_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
controller__srv__LinearWrench_Response__init(controller__srv__LinearWrench_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  return true;
}

void
controller__srv__LinearWrench_Response__fini(controller__srv__LinearWrench_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
}

bool
controller__srv__LinearWrench_Response__are_equal(const controller__srv__LinearWrench_Response * lhs, const controller__srv__LinearWrench_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  return true;
}

bool
controller__srv__LinearWrench_Response__copy(
  const controller__srv__LinearWrench_Response * input,
  controller__srv__LinearWrench_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  return true;
}

controller__srv__LinearWrench_Response *
controller__srv__LinearWrench_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__srv__LinearWrench_Response * msg = (controller__srv__LinearWrench_Response *)allocator.allocate(sizeof(controller__srv__LinearWrench_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(controller__srv__LinearWrench_Response));
  bool success = controller__srv__LinearWrench_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
controller__srv__LinearWrench_Response__destroy(controller__srv__LinearWrench_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    controller__srv__LinearWrench_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
controller__srv__LinearWrench_Response__Sequence__init(controller__srv__LinearWrench_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__srv__LinearWrench_Response * data = NULL;

  if (size) {
    data = (controller__srv__LinearWrench_Response *)allocator.zero_allocate(size, sizeof(controller__srv__LinearWrench_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = controller__srv__LinearWrench_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        controller__srv__LinearWrench_Response__fini(&data[i - 1]);
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
controller__srv__LinearWrench_Response__Sequence__fini(controller__srv__LinearWrench_Response__Sequence * array)
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
      controller__srv__LinearWrench_Response__fini(&array->data[i]);
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

controller__srv__LinearWrench_Response__Sequence *
controller__srv__LinearWrench_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__srv__LinearWrench_Response__Sequence * array = (controller__srv__LinearWrench_Response__Sequence *)allocator.allocate(sizeof(controller__srv__LinearWrench_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = controller__srv__LinearWrench_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
controller__srv__LinearWrench_Response__Sequence__destroy(controller__srv__LinearWrench_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    controller__srv__LinearWrench_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
controller__srv__LinearWrench_Response__Sequence__are_equal(const controller__srv__LinearWrench_Response__Sequence * lhs, const controller__srv__LinearWrench_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!controller__srv__LinearWrench_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
controller__srv__LinearWrench_Response__Sequence__copy(
  const controller__srv__LinearWrench_Response__Sequence * input,
  controller__srv__LinearWrench_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(controller__srv__LinearWrench_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    controller__srv__LinearWrench_Response * data =
      (controller__srv__LinearWrench_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!controller__srv__LinearWrench_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          controller__srv__LinearWrench_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!controller__srv__LinearWrench_Response__copy(
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
// #include "controller/srv/detail/linear_wrench__functions.h"

bool
controller__srv__LinearWrench_Event__init(controller__srv__LinearWrench_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    controller__srv__LinearWrench_Event__fini(msg);
    return false;
  }
  // request
  if (!controller__srv__LinearWrench_Request__Sequence__init(&msg->request, 0)) {
    controller__srv__LinearWrench_Event__fini(msg);
    return false;
  }
  // response
  if (!controller__srv__LinearWrench_Response__Sequence__init(&msg->response, 0)) {
    controller__srv__LinearWrench_Event__fini(msg);
    return false;
  }
  return true;
}

void
controller__srv__LinearWrench_Event__fini(controller__srv__LinearWrench_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  controller__srv__LinearWrench_Request__Sequence__fini(&msg->request);
  // response
  controller__srv__LinearWrench_Response__Sequence__fini(&msg->response);
}

bool
controller__srv__LinearWrench_Event__are_equal(const controller__srv__LinearWrench_Event * lhs, const controller__srv__LinearWrench_Event * rhs)
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
  if (!controller__srv__LinearWrench_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!controller__srv__LinearWrench_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
controller__srv__LinearWrench_Event__copy(
  const controller__srv__LinearWrench_Event * input,
  controller__srv__LinearWrench_Event * output)
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
  if (!controller__srv__LinearWrench_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!controller__srv__LinearWrench_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

controller__srv__LinearWrench_Event *
controller__srv__LinearWrench_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__srv__LinearWrench_Event * msg = (controller__srv__LinearWrench_Event *)allocator.allocate(sizeof(controller__srv__LinearWrench_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(controller__srv__LinearWrench_Event));
  bool success = controller__srv__LinearWrench_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
controller__srv__LinearWrench_Event__destroy(controller__srv__LinearWrench_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    controller__srv__LinearWrench_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
controller__srv__LinearWrench_Event__Sequence__init(controller__srv__LinearWrench_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__srv__LinearWrench_Event * data = NULL;

  if (size) {
    data = (controller__srv__LinearWrench_Event *)allocator.zero_allocate(size, sizeof(controller__srv__LinearWrench_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = controller__srv__LinearWrench_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        controller__srv__LinearWrench_Event__fini(&data[i - 1]);
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
controller__srv__LinearWrench_Event__Sequence__fini(controller__srv__LinearWrench_Event__Sequence * array)
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
      controller__srv__LinearWrench_Event__fini(&array->data[i]);
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

controller__srv__LinearWrench_Event__Sequence *
controller__srv__LinearWrench_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__srv__LinearWrench_Event__Sequence * array = (controller__srv__LinearWrench_Event__Sequence *)allocator.allocate(sizeof(controller__srv__LinearWrench_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = controller__srv__LinearWrench_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
controller__srv__LinearWrench_Event__Sequence__destroy(controller__srv__LinearWrench_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    controller__srv__LinearWrench_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
controller__srv__LinearWrench_Event__Sequence__are_equal(const controller__srv__LinearWrench_Event__Sequence * lhs, const controller__srv__LinearWrench_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!controller__srv__LinearWrench_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
controller__srv__LinearWrench_Event__Sequence__copy(
  const controller__srv__LinearWrench_Event__Sequence * input,
  controller__srv__LinearWrench_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(controller__srv__LinearWrench_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    controller__srv__LinearWrench_Event * data =
      (controller__srv__LinearWrench_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!controller__srv__LinearWrench_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          controller__srv__LinearWrench_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!controller__srv__LinearWrench_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
