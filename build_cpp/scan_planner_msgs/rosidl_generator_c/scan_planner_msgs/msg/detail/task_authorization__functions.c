// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from scan_planner_msgs:msg/TaskAuthorization.idl
// generated code does not contain a copyright notice
#include "scan_planner_msgs/msg/detail/task_authorization__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
scan_planner_msgs__msg__TaskAuthorization__init(scan_planner_msgs__msg__TaskAuthorization * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    scan_planner_msgs__msg__TaskAuthorization__fini(msg);
    return false;
  }
  // active
  // task_id
  return true;
}

void
scan_planner_msgs__msg__TaskAuthorization__fini(scan_planner_msgs__msg__TaskAuthorization * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // active
  // task_id
}

bool
scan_planner_msgs__msg__TaskAuthorization__are_equal(const scan_planner_msgs__msg__TaskAuthorization * lhs, const scan_planner_msgs__msg__TaskAuthorization * rhs)
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
  // active
  if (lhs->active != rhs->active) {
    return false;
  }
  // task_id
  if (lhs->task_id != rhs->task_id) {
    return false;
  }
  return true;
}

bool
scan_planner_msgs__msg__TaskAuthorization__copy(
  const scan_planner_msgs__msg__TaskAuthorization * input,
  scan_planner_msgs__msg__TaskAuthorization * output)
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
  // active
  output->active = input->active;
  // task_id
  output->task_id = input->task_id;
  return true;
}

scan_planner_msgs__msg__TaskAuthorization *
scan_planner_msgs__msg__TaskAuthorization__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scan_planner_msgs__msg__TaskAuthorization * msg = (scan_planner_msgs__msg__TaskAuthorization *)allocator.allocate(sizeof(scan_planner_msgs__msg__TaskAuthorization), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(scan_planner_msgs__msg__TaskAuthorization));
  bool success = scan_planner_msgs__msg__TaskAuthorization__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
scan_planner_msgs__msg__TaskAuthorization__destroy(scan_planner_msgs__msg__TaskAuthorization * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    scan_planner_msgs__msg__TaskAuthorization__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
scan_planner_msgs__msg__TaskAuthorization__Sequence__init(scan_planner_msgs__msg__TaskAuthorization__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scan_planner_msgs__msg__TaskAuthorization * data = NULL;

  if (size) {
    data = (scan_planner_msgs__msg__TaskAuthorization *)allocator.zero_allocate(size, sizeof(scan_planner_msgs__msg__TaskAuthorization), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = scan_planner_msgs__msg__TaskAuthorization__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        scan_planner_msgs__msg__TaskAuthorization__fini(&data[i - 1]);
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
scan_planner_msgs__msg__TaskAuthorization__Sequence__fini(scan_planner_msgs__msg__TaskAuthorization__Sequence * array)
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
      scan_planner_msgs__msg__TaskAuthorization__fini(&array->data[i]);
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

scan_planner_msgs__msg__TaskAuthorization__Sequence *
scan_planner_msgs__msg__TaskAuthorization__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scan_planner_msgs__msg__TaskAuthorization__Sequence * array = (scan_planner_msgs__msg__TaskAuthorization__Sequence *)allocator.allocate(sizeof(scan_planner_msgs__msg__TaskAuthorization__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = scan_planner_msgs__msg__TaskAuthorization__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
scan_planner_msgs__msg__TaskAuthorization__Sequence__destroy(scan_planner_msgs__msg__TaskAuthorization__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    scan_planner_msgs__msg__TaskAuthorization__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
scan_planner_msgs__msg__TaskAuthorization__Sequence__are_equal(const scan_planner_msgs__msg__TaskAuthorization__Sequence * lhs, const scan_planner_msgs__msg__TaskAuthorization__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!scan_planner_msgs__msg__TaskAuthorization__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
scan_planner_msgs__msg__TaskAuthorization__Sequence__copy(
  const scan_planner_msgs__msg__TaskAuthorization__Sequence * input,
  scan_planner_msgs__msg__TaskAuthorization__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(scan_planner_msgs__msg__TaskAuthorization);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    scan_planner_msgs__msg__TaskAuthorization * data =
      (scan_planner_msgs__msg__TaskAuthorization *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!scan_planner_msgs__msg__TaskAuthorization__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          scan_planner_msgs__msg__TaskAuthorization__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!scan_planner_msgs__msg__TaskAuthorization__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
