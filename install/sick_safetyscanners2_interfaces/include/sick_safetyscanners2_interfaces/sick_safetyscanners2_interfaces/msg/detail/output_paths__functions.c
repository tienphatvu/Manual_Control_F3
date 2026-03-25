// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from sick_safetyscanners2_interfaces:msg/OutputPaths.idl
// generated code does not contain a copyright notice
#include "sick_safetyscanners2_interfaces/msg/detail/output_paths__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `status`
// Member `is_safe`
// Member `is_valid`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
sick_safetyscanners2_interfaces__msg__OutputPaths__init(sick_safetyscanners2_interfaces__msg__OutputPaths * msg)
{
  if (!msg) {
    return false;
  }
  // status
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->status, 0)) {
    sick_safetyscanners2_interfaces__msg__OutputPaths__fini(msg);
    return false;
  }
  // is_safe
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->is_safe, 0)) {
    sick_safetyscanners2_interfaces__msg__OutputPaths__fini(msg);
    return false;
  }
  // is_valid
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->is_valid, 0)) {
    sick_safetyscanners2_interfaces__msg__OutputPaths__fini(msg);
    return false;
  }
  // active_monitoring_case
  return true;
}

void
sick_safetyscanners2_interfaces__msg__OutputPaths__fini(sick_safetyscanners2_interfaces__msg__OutputPaths * msg)
{
  if (!msg) {
    return;
  }
  // status
  rosidl_runtime_c__boolean__Sequence__fini(&msg->status);
  // is_safe
  rosidl_runtime_c__boolean__Sequence__fini(&msg->is_safe);
  // is_valid
  rosidl_runtime_c__boolean__Sequence__fini(&msg->is_valid);
  // active_monitoring_case
}

bool
sick_safetyscanners2_interfaces__msg__OutputPaths__are_equal(const sick_safetyscanners2_interfaces__msg__OutputPaths * lhs, const sick_safetyscanners2_interfaces__msg__OutputPaths * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->status), &(rhs->status)))
  {
    return false;
  }
  // is_safe
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->is_safe), &(rhs->is_safe)))
  {
    return false;
  }
  // is_valid
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->is_valid), &(rhs->is_valid)))
  {
    return false;
  }
  // active_monitoring_case
  if (lhs->active_monitoring_case != rhs->active_monitoring_case) {
    return false;
  }
  return true;
}

bool
sick_safetyscanners2_interfaces__msg__OutputPaths__copy(
  const sick_safetyscanners2_interfaces__msg__OutputPaths * input,
  sick_safetyscanners2_interfaces__msg__OutputPaths * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->status), &(output->status)))
  {
    return false;
  }
  // is_safe
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->is_safe), &(output->is_safe)))
  {
    return false;
  }
  // is_valid
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->is_valid), &(output->is_valid)))
  {
    return false;
  }
  // active_monitoring_case
  output->active_monitoring_case = input->active_monitoring_case;
  return true;
}

sick_safetyscanners2_interfaces__msg__OutputPaths *
sick_safetyscanners2_interfaces__msg__OutputPaths__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__OutputPaths * msg = (sick_safetyscanners2_interfaces__msg__OutputPaths *)allocator.allocate(sizeof(sick_safetyscanners2_interfaces__msg__OutputPaths), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sick_safetyscanners2_interfaces__msg__OutputPaths));
  bool success = sick_safetyscanners2_interfaces__msg__OutputPaths__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sick_safetyscanners2_interfaces__msg__OutputPaths__destroy(sick_safetyscanners2_interfaces__msg__OutputPaths * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sick_safetyscanners2_interfaces__msg__OutputPaths__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence__init(sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__OutputPaths * data = NULL;

  if (size) {
    data = (sick_safetyscanners2_interfaces__msg__OutputPaths *)allocator.zero_allocate(size, sizeof(sick_safetyscanners2_interfaces__msg__OutputPaths), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sick_safetyscanners2_interfaces__msg__OutputPaths__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sick_safetyscanners2_interfaces__msg__OutputPaths__fini(&data[i - 1]);
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
sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence__fini(sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence * array)
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
      sick_safetyscanners2_interfaces__msg__OutputPaths__fini(&array->data[i]);
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

sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence *
sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence * array = (sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence *)allocator.allocate(sizeof(sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence__destroy(sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence__are_equal(const sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence * lhs, const sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sick_safetyscanners2_interfaces__msg__OutputPaths__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence__copy(
  const sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence * input,
  sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sick_safetyscanners2_interfaces__msg__OutputPaths);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sick_safetyscanners2_interfaces__msg__OutputPaths * data =
      (sick_safetyscanners2_interfaces__msg__OutputPaths *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sick_safetyscanners2_interfaces__msg__OutputPaths__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sick_safetyscanners2_interfaces__msg__OutputPaths__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sick_safetyscanners2_interfaces__msg__OutputPaths__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
