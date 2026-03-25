// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from sick_safetyscanners2_interfaces:msg/RawMicroScanData.idl
// generated code does not contain a copyright notice
#include "sick_safetyscanners2_interfaces/msg/detail/raw_micro_scan_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "sick_safetyscanners2_interfaces/msg/detail/data_header__functions.h"
// Member `derived_values`
#include "sick_safetyscanners2_interfaces/msg/detail/derived_values__functions.h"
// Member `general_system_state`
#include "sick_safetyscanners2_interfaces/msg/detail/general_system_state__functions.h"
// Member `measurement_data`
#include "sick_safetyscanners2_interfaces/msg/detail/measurement_data__functions.h"
// Member `intrusion_data`
#include "sick_safetyscanners2_interfaces/msg/detail/intrusion_data__functions.h"
// Member `application_data`
#include "sick_safetyscanners2_interfaces/msg/detail/application_data__functions.h"

bool
sick_safetyscanners2_interfaces__msg__RawMicroScanData__init(sick_safetyscanners2_interfaces__msg__RawMicroScanData * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!sick_safetyscanners2_interfaces__msg__DataHeader__init(&msg->header)) {
    sick_safetyscanners2_interfaces__msg__RawMicroScanData__fini(msg);
    return false;
  }
  // derived_values
  if (!sick_safetyscanners2_interfaces__msg__DerivedValues__init(&msg->derived_values)) {
    sick_safetyscanners2_interfaces__msg__RawMicroScanData__fini(msg);
    return false;
  }
  // general_system_state
  if (!sick_safetyscanners2_interfaces__msg__GeneralSystemState__init(&msg->general_system_state)) {
    sick_safetyscanners2_interfaces__msg__RawMicroScanData__fini(msg);
    return false;
  }
  // measurement_data
  if (!sick_safetyscanners2_interfaces__msg__MeasurementData__init(&msg->measurement_data)) {
    sick_safetyscanners2_interfaces__msg__RawMicroScanData__fini(msg);
    return false;
  }
  // intrusion_data
  if (!sick_safetyscanners2_interfaces__msg__IntrusionData__init(&msg->intrusion_data)) {
    sick_safetyscanners2_interfaces__msg__RawMicroScanData__fini(msg);
    return false;
  }
  // application_data
  if (!sick_safetyscanners2_interfaces__msg__ApplicationData__init(&msg->application_data)) {
    sick_safetyscanners2_interfaces__msg__RawMicroScanData__fini(msg);
    return false;
  }
  return true;
}

void
sick_safetyscanners2_interfaces__msg__RawMicroScanData__fini(sick_safetyscanners2_interfaces__msg__RawMicroScanData * msg)
{
  if (!msg) {
    return;
  }
  // header
  sick_safetyscanners2_interfaces__msg__DataHeader__fini(&msg->header);
  // derived_values
  sick_safetyscanners2_interfaces__msg__DerivedValues__fini(&msg->derived_values);
  // general_system_state
  sick_safetyscanners2_interfaces__msg__GeneralSystemState__fini(&msg->general_system_state);
  // measurement_data
  sick_safetyscanners2_interfaces__msg__MeasurementData__fini(&msg->measurement_data);
  // intrusion_data
  sick_safetyscanners2_interfaces__msg__IntrusionData__fini(&msg->intrusion_data);
  // application_data
  sick_safetyscanners2_interfaces__msg__ApplicationData__fini(&msg->application_data);
}

bool
sick_safetyscanners2_interfaces__msg__RawMicroScanData__are_equal(const sick_safetyscanners2_interfaces__msg__RawMicroScanData * lhs, const sick_safetyscanners2_interfaces__msg__RawMicroScanData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!sick_safetyscanners2_interfaces__msg__DataHeader__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // derived_values
  if (!sick_safetyscanners2_interfaces__msg__DerivedValues__are_equal(
      &(lhs->derived_values), &(rhs->derived_values)))
  {
    return false;
  }
  // general_system_state
  if (!sick_safetyscanners2_interfaces__msg__GeneralSystemState__are_equal(
      &(lhs->general_system_state), &(rhs->general_system_state)))
  {
    return false;
  }
  // measurement_data
  if (!sick_safetyscanners2_interfaces__msg__MeasurementData__are_equal(
      &(lhs->measurement_data), &(rhs->measurement_data)))
  {
    return false;
  }
  // intrusion_data
  if (!sick_safetyscanners2_interfaces__msg__IntrusionData__are_equal(
      &(lhs->intrusion_data), &(rhs->intrusion_data)))
  {
    return false;
  }
  // application_data
  if (!sick_safetyscanners2_interfaces__msg__ApplicationData__are_equal(
      &(lhs->application_data), &(rhs->application_data)))
  {
    return false;
  }
  return true;
}

bool
sick_safetyscanners2_interfaces__msg__RawMicroScanData__copy(
  const sick_safetyscanners2_interfaces__msg__RawMicroScanData * input,
  sick_safetyscanners2_interfaces__msg__RawMicroScanData * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!sick_safetyscanners2_interfaces__msg__DataHeader__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // derived_values
  if (!sick_safetyscanners2_interfaces__msg__DerivedValues__copy(
      &(input->derived_values), &(output->derived_values)))
  {
    return false;
  }
  // general_system_state
  if (!sick_safetyscanners2_interfaces__msg__GeneralSystemState__copy(
      &(input->general_system_state), &(output->general_system_state)))
  {
    return false;
  }
  // measurement_data
  if (!sick_safetyscanners2_interfaces__msg__MeasurementData__copy(
      &(input->measurement_data), &(output->measurement_data)))
  {
    return false;
  }
  // intrusion_data
  if (!sick_safetyscanners2_interfaces__msg__IntrusionData__copy(
      &(input->intrusion_data), &(output->intrusion_data)))
  {
    return false;
  }
  // application_data
  if (!sick_safetyscanners2_interfaces__msg__ApplicationData__copy(
      &(input->application_data), &(output->application_data)))
  {
    return false;
  }
  return true;
}

sick_safetyscanners2_interfaces__msg__RawMicroScanData *
sick_safetyscanners2_interfaces__msg__RawMicroScanData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__RawMicroScanData * msg = (sick_safetyscanners2_interfaces__msg__RawMicroScanData *)allocator.allocate(sizeof(sick_safetyscanners2_interfaces__msg__RawMicroScanData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sick_safetyscanners2_interfaces__msg__RawMicroScanData));
  bool success = sick_safetyscanners2_interfaces__msg__RawMicroScanData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sick_safetyscanners2_interfaces__msg__RawMicroScanData__destroy(sick_safetyscanners2_interfaces__msg__RawMicroScanData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sick_safetyscanners2_interfaces__msg__RawMicroScanData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sick_safetyscanners2_interfaces__msg__RawMicroScanData__Sequence__init(sick_safetyscanners2_interfaces__msg__RawMicroScanData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__RawMicroScanData * data = NULL;

  if (size) {
    data = (sick_safetyscanners2_interfaces__msg__RawMicroScanData *)allocator.zero_allocate(size, sizeof(sick_safetyscanners2_interfaces__msg__RawMicroScanData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sick_safetyscanners2_interfaces__msg__RawMicroScanData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sick_safetyscanners2_interfaces__msg__RawMicroScanData__fini(&data[i - 1]);
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
sick_safetyscanners2_interfaces__msg__RawMicroScanData__Sequence__fini(sick_safetyscanners2_interfaces__msg__RawMicroScanData__Sequence * array)
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
      sick_safetyscanners2_interfaces__msg__RawMicroScanData__fini(&array->data[i]);
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

sick_safetyscanners2_interfaces__msg__RawMicroScanData__Sequence *
sick_safetyscanners2_interfaces__msg__RawMicroScanData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__RawMicroScanData__Sequence * array = (sick_safetyscanners2_interfaces__msg__RawMicroScanData__Sequence *)allocator.allocate(sizeof(sick_safetyscanners2_interfaces__msg__RawMicroScanData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sick_safetyscanners2_interfaces__msg__RawMicroScanData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sick_safetyscanners2_interfaces__msg__RawMicroScanData__Sequence__destroy(sick_safetyscanners2_interfaces__msg__RawMicroScanData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sick_safetyscanners2_interfaces__msg__RawMicroScanData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sick_safetyscanners2_interfaces__msg__RawMicroScanData__Sequence__are_equal(const sick_safetyscanners2_interfaces__msg__RawMicroScanData__Sequence * lhs, const sick_safetyscanners2_interfaces__msg__RawMicroScanData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sick_safetyscanners2_interfaces__msg__RawMicroScanData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sick_safetyscanners2_interfaces__msg__RawMicroScanData__Sequence__copy(
  const sick_safetyscanners2_interfaces__msg__RawMicroScanData__Sequence * input,
  sick_safetyscanners2_interfaces__msg__RawMicroScanData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sick_safetyscanners2_interfaces__msg__RawMicroScanData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sick_safetyscanners2_interfaces__msg__RawMicroScanData * data =
      (sick_safetyscanners2_interfaces__msg__RawMicroScanData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sick_safetyscanners2_interfaces__msg__RawMicroScanData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sick_safetyscanners2_interfaces__msg__RawMicroScanData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sick_safetyscanners2_interfaces__msg__RawMicroScanData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
