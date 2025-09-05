// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hardware_monitor2_interfaces:srv/Logging.idl
// generated code does not contain a copyright notice
#include "hardware_monitor2_interfaces/srv/detail/logging__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `is_logging`
#include "rosidl_runtime_c/string_functions.h"

bool
hardware_monitor2_interfaces__srv__Logging_Request__init(hardware_monitor2_interfaces__srv__Logging_Request * msg)
{
  if (!msg) {
    return false;
  }
  // is_logging
  if (!rosidl_runtime_c__String__init(&msg->is_logging)) {
    hardware_monitor2_interfaces__srv__Logging_Request__fini(msg);
    return false;
  }
  return true;
}

void
hardware_monitor2_interfaces__srv__Logging_Request__fini(hardware_monitor2_interfaces__srv__Logging_Request * msg)
{
  if (!msg) {
    return;
  }
  // is_logging
  rosidl_runtime_c__String__fini(&msg->is_logging);
}

bool
hardware_monitor2_interfaces__srv__Logging_Request__are_equal(const hardware_monitor2_interfaces__srv__Logging_Request * lhs, const hardware_monitor2_interfaces__srv__Logging_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // is_logging
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->is_logging), &(rhs->is_logging)))
  {
    return false;
  }
  return true;
}

bool
hardware_monitor2_interfaces__srv__Logging_Request__copy(
  const hardware_monitor2_interfaces__srv__Logging_Request * input,
  hardware_monitor2_interfaces__srv__Logging_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // is_logging
  if (!rosidl_runtime_c__String__copy(
      &(input->is_logging), &(output->is_logging)))
  {
    return false;
  }
  return true;
}

hardware_monitor2_interfaces__srv__Logging_Request *
hardware_monitor2_interfaces__srv__Logging_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hardware_monitor2_interfaces__srv__Logging_Request * msg = (hardware_monitor2_interfaces__srv__Logging_Request *)allocator.allocate(sizeof(hardware_monitor2_interfaces__srv__Logging_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hardware_monitor2_interfaces__srv__Logging_Request));
  bool success = hardware_monitor2_interfaces__srv__Logging_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hardware_monitor2_interfaces__srv__Logging_Request__destroy(hardware_monitor2_interfaces__srv__Logging_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hardware_monitor2_interfaces__srv__Logging_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hardware_monitor2_interfaces__srv__Logging_Request__Sequence__init(hardware_monitor2_interfaces__srv__Logging_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hardware_monitor2_interfaces__srv__Logging_Request * data = NULL;

  if (size) {
    data = (hardware_monitor2_interfaces__srv__Logging_Request *)allocator.zero_allocate(size, sizeof(hardware_monitor2_interfaces__srv__Logging_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hardware_monitor2_interfaces__srv__Logging_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hardware_monitor2_interfaces__srv__Logging_Request__fini(&data[i - 1]);
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
hardware_monitor2_interfaces__srv__Logging_Request__Sequence__fini(hardware_monitor2_interfaces__srv__Logging_Request__Sequence * array)
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
      hardware_monitor2_interfaces__srv__Logging_Request__fini(&array->data[i]);
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

hardware_monitor2_interfaces__srv__Logging_Request__Sequence *
hardware_monitor2_interfaces__srv__Logging_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hardware_monitor2_interfaces__srv__Logging_Request__Sequence * array = (hardware_monitor2_interfaces__srv__Logging_Request__Sequence *)allocator.allocate(sizeof(hardware_monitor2_interfaces__srv__Logging_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hardware_monitor2_interfaces__srv__Logging_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hardware_monitor2_interfaces__srv__Logging_Request__Sequence__destroy(hardware_monitor2_interfaces__srv__Logging_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hardware_monitor2_interfaces__srv__Logging_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hardware_monitor2_interfaces__srv__Logging_Request__Sequence__are_equal(const hardware_monitor2_interfaces__srv__Logging_Request__Sequence * lhs, const hardware_monitor2_interfaces__srv__Logging_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hardware_monitor2_interfaces__srv__Logging_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hardware_monitor2_interfaces__srv__Logging_Request__Sequence__copy(
  const hardware_monitor2_interfaces__srv__Logging_Request__Sequence * input,
  hardware_monitor2_interfaces__srv__Logging_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hardware_monitor2_interfaces__srv__Logging_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hardware_monitor2_interfaces__srv__Logging_Request * data =
      (hardware_monitor2_interfaces__srv__Logging_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hardware_monitor2_interfaces__srv__Logging_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hardware_monitor2_interfaces__srv__Logging_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hardware_monitor2_interfaces__srv__Logging_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `logging_status`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
hardware_monitor2_interfaces__srv__Logging_Response__init(hardware_monitor2_interfaces__srv__Logging_Response * msg)
{
  if (!msg) {
    return false;
  }
  // logging_status
  if (!rosidl_runtime_c__String__init(&msg->logging_status)) {
    hardware_monitor2_interfaces__srv__Logging_Response__fini(msg);
    return false;
  }
  return true;
}

void
hardware_monitor2_interfaces__srv__Logging_Response__fini(hardware_monitor2_interfaces__srv__Logging_Response * msg)
{
  if (!msg) {
    return;
  }
  // logging_status
  rosidl_runtime_c__String__fini(&msg->logging_status);
}

bool
hardware_monitor2_interfaces__srv__Logging_Response__are_equal(const hardware_monitor2_interfaces__srv__Logging_Response * lhs, const hardware_monitor2_interfaces__srv__Logging_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // logging_status
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->logging_status), &(rhs->logging_status)))
  {
    return false;
  }
  return true;
}

bool
hardware_monitor2_interfaces__srv__Logging_Response__copy(
  const hardware_monitor2_interfaces__srv__Logging_Response * input,
  hardware_monitor2_interfaces__srv__Logging_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // logging_status
  if (!rosidl_runtime_c__String__copy(
      &(input->logging_status), &(output->logging_status)))
  {
    return false;
  }
  return true;
}

hardware_monitor2_interfaces__srv__Logging_Response *
hardware_monitor2_interfaces__srv__Logging_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hardware_monitor2_interfaces__srv__Logging_Response * msg = (hardware_monitor2_interfaces__srv__Logging_Response *)allocator.allocate(sizeof(hardware_monitor2_interfaces__srv__Logging_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hardware_monitor2_interfaces__srv__Logging_Response));
  bool success = hardware_monitor2_interfaces__srv__Logging_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hardware_monitor2_interfaces__srv__Logging_Response__destroy(hardware_monitor2_interfaces__srv__Logging_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hardware_monitor2_interfaces__srv__Logging_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hardware_monitor2_interfaces__srv__Logging_Response__Sequence__init(hardware_monitor2_interfaces__srv__Logging_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hardware_monitor2_interfaces__srv__Logging_Response * data = NULL;

  if (size) {
    data = (hardware_monitor2_interfaces__srv__Logging_Response *)allocator.zero_allocate(size, sizeof(hardware_monitor2_interfaces__srv__Logging_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hardware_monitor2_interfaces__srv__Logging_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hardware_monitor2_interfaces__srv__Logging_Response__fini(&data[i - 1]);
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
hardware_monitor2_interfaces__srv__Logging_Response__Sequence__fini(hardware_monitor2_interfaces__srv__Logging_Response__Sequence * array)
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
      hardware_monitor2_interfaces__srv__Logging_Response__fini(&array->data[i]);
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

hardware_monitor2_interfaces__srv__Logging_Response__Sequence *
hardware_monitor2_interfaces__srv__Logging_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hardware_monitor2_interfaces__srv__Logging_Response__Sequence * array = (hardware_monitor2_interfaces__srv__Logging_Response__Sequence *)allocator.allocate(sizeof(hardware_monitor2_interfaces__srv__Logging_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hardware_monitor2_interfaces__srv__Logging_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hardware_monitor2_interfaces__srv__Logging_Response__Sequence__destroy(hardware_monitor2_interfaces__srv__Logging_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hardware_monitor2_interfaces__srv__Logging_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hardware_monitor2_interfaces__srv__Logging_Response__Sequence__are_equal(const hardware_monitor2_interfaces__srv__Logging_Response__Sequence * lhs, const hardware_monitor2_interfaces__srv__Logging_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hardware_monitor2_interfaces__srv__Logging_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hardware_monitor2_interfaces__srv__Logging_Response__Sequence__copy(
  const hardware_monitor2_interfaces__srv__Logging_Response__Sequence * input,
  hardware_monitor2_interfaces__srv__Logging_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hardware_monitor2_interfaces__srv__Logging_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hardware_monitor2_interfaces__srv__Logging_Response * data =
      (hardware_monitor2_interfaces__srv__Logging_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hardware_monitor2_interfaces__srv__Logging_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hardware_monitor2_interfaces__srv__Logging_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hardware_monitor2_interfaces__srv__Logging_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
