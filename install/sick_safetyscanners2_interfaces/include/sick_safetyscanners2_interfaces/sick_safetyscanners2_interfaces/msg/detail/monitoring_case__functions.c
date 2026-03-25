// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from sick_safetyscanners2_interfaces:msg/MonitoringCase.idl
// generated code does not contain a copyright notice
#include "sick_safetyscanners2_interfaces/msg/detail/monitoring_case__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `fields`
// Member `fields_valid`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
sick_safetyscanners2_interfaces__msg__MonitoringCase__init(sick_safetyscanners2_interfaces__msg__MonitoringCase * msg)
{
  if (!msg) {
    return false;
  }
  // monitoring_case_number
  // fields
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->fields, 0)) {
    sick_safetyscanners2_interfaces__msg__MonitoringCase__fini(msg);
    return false;
  }
  // fields_valid
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->fields_valid, 0)) {
    sick_safetyscanners2_interfaces__msg__MonitoringCase__fini(msg);
    return false;
  }
  return true;
}

void
sick_safetyscanners2_interfaces__msg__MonitoringCase__fini(sick_safetyscanners2_interfaces__msg__MonitoringCase * msg)
{
  if (!msg) {
    return;
  }
  // monitoring_case_number
  // fields
  rosidl_runtime_c__int32__Sequence__fini(&msg->fields);
  // fields_valid
  rosidl_runtime_c__boolean__Sequence__fini(&msg->fields_valid);
}

bool
sick_safetyscanners2_interfaces__msg__MonitoringCase__are_equal(const sick_safetyscanners2_interfaces__msg__MonitoringCase * lhs, const sick_safetyscanners2_interfaces__msg__MonitoringCase * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // monitoring_case_number
  if (lhs->monitoring_case_number != rhs->monitoring_case_number) {
    return false;
  }
  // fields
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->fields), &(rhs->fields)))
  {
    return false;
  }
  // fields_valid
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->fields_valid), &(rhs->fields_valid)))
  {
    return false;
  }
  return true;
}

bool
sick_safetyscanners2_interfaces__msg__MonitoringCase__copy(
  const sick_safetyscanners2_interfaces__msg__MonitoringCase * input,
  sick_safetyscanners2_interfaces__msg__MonitoringCase * output)
{
  if (!input || !output) {
    return false;
  }
  // monitoring_case_number
  output->monitoring_case_number = input->monitoring_case_number;
  // fields
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->fields), &(output->fields)))
  {
    return false;
  }
  // fields_valid
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->fields_valid), &(output->fields_valid)))
  {
    return false;
  }
  return true;
}

sick_safetyscanners2_interfaces__msg__MonitoringCase *
sick_safetyscanners2_interfaces__msg__MonitoringCase__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__MonitoringCase * msg = (sick_safetyscanners2_interfaces__msg__MonitoringCase *)allocator.allocate(sizeof(sick_safetyscanners2_interfaces__msg__MonitoringCase), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sick_safetyscanners2_interfaces__msg__MonitoringCase));
  bool success = sick_safetyscanners2_interfaces__msg__MonitoringCase__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sick_safetyscanners2_interfaces__msg__MonitoringCase__destroy(sick_safetyscanners2_interfaces__msg__MonitoringCase * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sick_safetyscanners2_interfaces__msg__MonitoringCase__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence__init(sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__MonitoringCase * data = NULL;

  if (size) {
    data = (sick_safetyscanners2_interfaces__msg__MonitoringCase *)allocator.zero_allocate(size, sizeof(sick_safetyscanners2_interfaces__msg__MonitoringCase), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sick_safetyscanners2_interfaces__msg__MonitoringCase__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sick_safetyscanners2_interfaces__msg__MonitoringCase__fini(&data[i - 1]);
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
sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence__fini(sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence * array)
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
      sick_safetyscanners2_interfaces__msg__MonitoringCase__fini(&array->data[i]);
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

sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence *
sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence * array = (sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence *)allocator.allocate(sizeof(sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence__destroy(sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence__are_equal(const sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence * lhs, const sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sick_safetyscanners2_interfaces__msg__MonitoringCase__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence__copy(
  const sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence * input,
  sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sick_safetyscanners2_interfaces__msg__MonitoringCase);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sick_safetyscanners2_interfaces__msg__MonitoringCase * data =
      (sick_safetyscanners2_interfaces__msg__MonitoringCase *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sick_safetyscanners2_interfaces__msg__MonitoringCase__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sick_safetyscanners2_interfaces__msg__MonitoringCase__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sick_safetyscanners2_interfaces__msg__MonitoringCase__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
