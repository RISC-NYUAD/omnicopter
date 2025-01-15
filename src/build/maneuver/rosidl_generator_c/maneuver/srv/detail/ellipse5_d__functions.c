// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from maneuver:srv/Ellipse5D.idl
// generated code does not contain a copyright notice
#include "maneuver/srv/detail/ellipse5_d__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
maneuver__srv__Ellipse5D_Request__init(maneuver__srv__Ellipse5D_Request * msg)
{
  if (!msg) {
    return false;
  }
  // x_min
  // x_max
  // y_min
  // y_max
  // z_min
  // z_max
  // roll_start
  // roll_mid
  // roll_end
  // pitch_start
  // pitch_mid
  // pitch_end
  // yaw
  // duration
  return true;
}

void
maneuver__srv__Ellipse5D_Request__fini(maneuver__srv__Ellipse5D_Request * msg)
{
  if (!msg) {
    return;
  }
  // x_min
  // x_max
  // y_min
  // y_max
  // z_min
  // z_max
  // roll_start
  // roll_mid
  // roll_end
  // pitch_start
  // pitch_mid
  // pitch_end
  // yaw
  // duration
}

bool
maneuver__srv__Ellipse5D_Request__are_equal(const maneuver__srv__Ellipse5D_Request * lhs, const maneuver__srv__Ellipse5D_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x_min
  if (lhs->x_min != rhs->x_min) {
    return false;
  }
  // x_max
  if (lhs->x_max != rhs->x_max) {
    return false;
  }
  // y_min
  if (lhs->y_min != rhs->y_min) {
    return false;
  }
  // y_max
  if (lhs->y_max != rhs->y_max) {
    return false;
  }
  // z_min
  if (lhs->z_min != rhs->z_min) {
    return false;
  }
  // z_max
  if (lhs->z_max != rhs->z_max) {
    return false;
  }
  // roll_start
  if (lhs->roll_start != rhs->roll_start) {
    return false;
  }
  // roll_mid
  if (lhs->roll_mid != rhs->roll_mid) {
    return false;
  }
  // roll_end
  if (lhs->roll_end != rhs->roll_end) {
    return false;
  }
  // pitch_start
  if (lhs->pitch_start != rhs->pitch_start) {
    return false;
  }
  // pitch_mid
  if (lhs->pitch_mid != rhs->pitch_mid) {
    return false;
  }
  // pitch_end
  if (lhs->pitch_end != rhs->pitch_end) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // duration
  if (lhs->duration != rhs->duration) {
    return false;
  }
  return true;
}

bool
maneuver__srv__Ellipse5D_Request__copy(
  const maneuver__srv__Ellipse5D_Request * input,
  maneuver__srv__Ellipse5D_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // x_min
  output->x_min = input->x_min;
  // x_max
  output->x_max = input->x_max;
  // y_min
  output->y_min = input->y_min;
  // y_max
  output->y_max = input->y_max;
  // z_min
  output->z_min = input->z_min;
  // z_max
  output->z_max = input->z_max;
  // roll_start
  output->roll_start = input->roll_start;
  // roll_mid
  output->roll_mid = input->roll_mid;
  // roll_end
  output->roll_end = input->roll_end;
  // pitch_start
  output->pitch_start = input->pitch_start;
  // pitch_mid
  output->pitch_mid = input->pitch_mid;
  // pitch_end
  output->pitch_end = input->pitch_end;
  // yaw
  output->yaw = input->yaw;
  // duration
  output->duration = input->duration;
  return true;
}

maneuver__srv__Ellipse5D_Request *
maneuver__srv__Ellipse5D_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  maneuver__srv__Ellipse5D_Request * msg = (maneuver__srv__Ellipse5D_Request *)allocator.allocate(sizeof(maneuver__srv__Ellipse5D_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(maneuver__srv__Ellipse5D_Request));
  bool success = maneuver__srv__Ellipse5D_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
maneuver__srv__Ellipse5D_Request__destroy(maneuver__srv__Ellipse5D_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    maneuver__srv__Ellipse5D_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
maneuver__srv__Ellipse5D_Request__Sequence__init(maneuver__srv__Ellipse5D_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  maneuver__srv__Ellipse5D_Request * data = NULL;

  if (size) {
    data = (maneuver__srv__Ellipse5D_Request *)allocator.zero_allocate(size, sizeof(maneuver__srv__Ellipse5D_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = maneuver__srv__Ellipse5D_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        maneuver__srv__Ellipse5D_Request__fini(&data[i - 1]);
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
maneuver__srv__Ellipse5D_Request__Sequence__fini(maneuver__srv__Ellipse5D_Request__Sequence * array)
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
      maneuver__srv__Ellipse5D_Request__fini(&array->data[i]);
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

maneuver__srv__Ellipse5D_Request__Sequence *
maneuver__srv__Ellipse5D_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  maneuver__srv__Ellipse5D_Request__Sequence * array = (maneuver__srv__Ellipse5D_Request__Sequence *)allocator.allocate(sizeof(maneuver__srv__Ellipse5D_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = maneuver__srv__Ellipse5D_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
maneuver__srv__Ellipse5D_Request__Sequence__destroy(maneuver__srv__Ellipse5D_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    maneuver__srv__Ellipse5D_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
maneuver__srv__Ellipse5D_Request__Sequence__are_equal(const maneuver__srv__Ellipse5D_Request__Sequence * lhs, const maneuver__srv__Ellipse5D_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!maneuver__srv__Ellipse5D_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
maneuver__srv__Ellipse5D_Request__Sequence__copy(
  const maneuver__srv__Ellipse5D_Request__Sequence * input,
  maneuver__srv__Ellipse5D_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(maneuver__srv__Ellipse5D_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    maneuver__srv__Ellipse5D_Request * data =
      (maneuver__srv__Ellipse5D_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!maneuver__srv__Ellipse5D_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          maneuver__srv__Ellipse5D_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!maneuver__srv__Ellipse5D_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
maneuver__srv__Ellipse5D_Response__init(maneuver__srv__Ellipse5D_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  return true;
}

void
maneuver__srv__Ellipse5D_Response__fini(maneuver__srv__Ellipse5D_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
}

bool
maneuver__srv__Ellipse5D_Response__are_equal(const maneuver__srv__Ellipse5D_Response * lhs, const maneuver__srv__Ellipse5D_Response * rhs)
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
maneuver__srv__Ellipse5D_Response__copy(
  const maneuver__srv__Ellipse5D_Response * input,
  maneuver__srv__Ellipse5D_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  return true;
}

maneuver__srv__Ellipse5D_Response *
maneuver__srv__Ellipse5D_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  maneuver__srv__Ellipse5D_Response * msg = (maneuver__srv__Ellipse5D_Response *)allocator.allocate(sizeof(maneuver__srv__Ellipse5D_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(maneuver__srv__Ellipse5D_Response));
  bool success = maneuver__srv__Ellipse5D_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
maneuver__srv__Ellipse5D_Response__destroy(maneuver__srv__Ellipse5D_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    maneuver__srv__Ellipse5D_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
maneuver__srv__Ellipse5D_Response__Sequence__init(maneuver__srv__Ellipse5D_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  maneuver__srv__Ellipse5D_Response * data = NULL;

  if (size) {
    data = (maneuver__srv__Ellipse5D_Response *)allocator.zero_allocate(size, sizeof(maneuver__srv__Ellipse5D_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = maneuver__srv__Ellipse5D_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        maneuver__srv__Ellipse5D_Response__fini(&data[i - 1]);
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
maneuver__srv__Ellipse5D_Response__Sequence__fini(maneuver__srv__Ellipse5D_Response__Sequence * array)
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
      maneuver__srv__Ellipse5D_Response__fini(&array->data[i]);
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

maneuver__srv__Ellipse5D_Response__Sequence *
maneuver__srv__Ellipse5D_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  maneuver__srv__Ellipse5D_Response__Sequence * array = (maneuver__srv__Ellipse5D_Response__Sequence *)allocator.allocate(sizeof(maneuver__srv__Ellipse5D_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = maneuver__srv__Ellipse5D_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
maneuver__srv__Ellipse5D_Response__Sequence__destroy(maneuver__srv__Ellipse5D_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    maneuver__srv__Ellipse5D_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
maneuver__srv__Ellipse5D_Response__Sequence__are_equal(const maneuver__srv__Ellipse5D_Response__Sequence * lhs, const maneuver__srv__Ellipse5D_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!maneuver__srv__Ellipse5D_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
maneuver__srv__Ellipse5D_Response__Sequence__copy(
  const maneuver__srv__Ellipse5D_Response__Sequence * input,
  maneuver__srv__Ellipse5D_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(maneuver__srv__Ellipse5D_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    maneuver__srv__Ellipse5D_Response * data =
      (maneuver__srv__Ellipse5D_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!maneuver__srv__Ellipse5D_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          maneuver__srv__Ellipse5D_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!maneuver__srv__Ellipse5D_Response__copy(
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
// #include "maneuver/srv/detail/ellipse5_d__functions.h"

bool
maneuver__srv__Ellipse5D_Event__init(maneuver__srv__Ellipse5D_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    maneuver__srv__Ellipse5D_Event__fini(msg);
    return false;
  }
  // request
  if (!maneuver__srv__Ellipse5D_Request__Sequence__init(&msg->request, 0)) {
    maneuver__srv__Ellipse5D_Event__fini(msg);
    return false;
  }
  // response
  if (!maneuver__srv__Ellipse5D_Response__Sequence__init(&msg->response, 0)) {
    maneuver__srv__Ellipse5D_Event__fini(msg);
    return false;
  }
  return true;
}

void
maneuver__srv__Ellipse5D_Event__fini(maneuver__srv__Ellipse5D_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  maneuver__srv__Ellipse5D_Request__Sequence__fini(&msg->request);
  // response
  maneuver__srv__Ellipse5D_Response__Sequence__fini(&msg->response);
}

bool
maneuver__srv__Ellipse5D_Event__are_equal(const maneuver__srv__Ellipse5D_Event * lhs, const maneuver__srv__Ellipse5D_Event * rhs)
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
  if (!maneuver__srv__Ellipse5D_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!maneuver__srv__Ellipse5D_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
maneuver__srv__Ellipse5D_Event__copy(
  const maneuver__srv__Ellipse5D_Event * input,
  maneuver__srv__Ellipse5D_Event * output)
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
  if (!maneuver__srv__Ellipse5D_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!maneuver__srv__Ellipse5D_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

maneuver__srv__Ellipse5D_Event *
maneuver__srv__Ellipse5D_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  maneuver__srv__Ellipse5D_Event * msg = (maneuver__srv__Ellipse5D_Event *)allocator.allocate(sizeof(maneuver__srv__Ellipse5D_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(maneuver__srv__Ellipse5D_Event));
  bool success = maneuver__srv__Ellipse5D_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
maneuver__srv__Ellipse5D_Event__destroy(maneuver__srv__Ellipse5D_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    maneuver__srv__Ellipse5D_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
maneuver__srv__Ellipse5D_Event__Sequence__init(maneuver__srv__Ellipse5D_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  maneuver__srv__Ellipse5D_Event * data = NULL;

  if (size) {
    data = (maneuver__srv__Ellipse5D_Event *)allocator.zero_allocate(size, sizeof(maneuver__srv__Ellipse5D_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = maneuver__srv__Ellipse5D_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        maneuver__srv__Ellipse5D_Event__fini(&data[i - 1]);
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
maneuver__srv__Ellipse5D_Event__Sequence__fini(maneuver__srv__Ellipse5D_Event__Sequence * array)
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
      maneuver__srv__Ellipse5D_Event__fini(&array->data[i]);
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

maneuver__srv__Ellipse5D_Event__Sequence *
maneuver__srv__Ellipse5D_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  maneuver__srv__Ellipse5D_Event__Sequence * array = (maneuver__srv__Ellipse5D_Event__Sequence *)allocator.allocate(sizeof(maneuver__srv__Ellipse5D_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = maneuver__srv__Ellipse5D_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
maneuver__srv__Ellipse5D_Event__Sequence__destroy(maneuver__srv__Ellipse5D_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    maneuver__srv__Ellipse5D_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
maneuver__srv__Ellipse5D_Event__Sequence__are_equal(const maneuver__srv__Ellipse5D_Event__Sequence * lhs, const maneuver__srv__Ellipse5D_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!maneuver__srv__Ellipse5D_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
maneuver__srv__Ellipse5D_Event__Sequence__copy(
  const maneuver__srv__Ellipse5D_Event__Sequence * input,
  maneuver__srv__Ellipse5D_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(maneuver__srv__Ellipse5D_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    maneuver__srv__Ellipse5D_Event * data =
      (maneuver__srv__Ellipse5D_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!maneuver__srv__Ellipse5D_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          maneuver__srv__Ellipse5D_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!maneuver__srv__Ellipse5D_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
