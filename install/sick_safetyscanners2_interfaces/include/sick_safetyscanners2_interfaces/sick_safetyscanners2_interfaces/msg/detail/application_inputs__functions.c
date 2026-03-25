// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from sick_safetyscanners2_interfaces:msg/ApplicationInputs.idl
// generated code does not contain a copyright notice
#include "sick_safetyscanners2_interfaces/msg/detail/application_inputs__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `unsafe_inputs_input_sources`
// Member `unsafe_inputs_flags`
// Member `monitoring_case_number_inputs`
// Member `monitoring_case_number_inputs_flags`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
sick_safetyscanners2_interfaces__msg__ApplicationInputs__init(sick_safetyscanners2_interfaces__msg__ApplicationInputs * msg)
{
  if (!msg) {
    return false;
  }
  // unsafe_inputs_input_sources
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->unsafe_inputs_input_sources, 0)) {
    sick_safetyscanners2_interfaces__msg__ApplicationInputs__fini(msg);
    return false;
  }
  // unsafe_inputs_flags
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->unsafe_inputs_flags, 0)) {
    sick_safetyscanners2_interfaces__msg__ApplicationInputs__fini(msg);
    return false;
  }
  // monitoring_case_number_inputs
  if (!rosidl_runtime_c__uint16__Sequence__init(&msg->monitoring_case_number_inputs, 0)) {
    sick_safetyscanners2_interfaces__msg__ApplicationInputs__fini(msg);
    return false;
  }
  // monitoring_case_number_inputs_flags
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->monitoring_case_number_inputs_flags, 0)) {
    sick_safetyscanners2_interfaces__msg__ApplicationInputs__fini(msg);
    return false;
  }
  // linear_velocity_inputs_velocity_0
  // linear_velocity_inputs_velocity_0_valid
  // linear_velocity_inputs_velocity_0_transmitted_safely
  // linear_velocity_inputs_velocity_1
  // linear_velocity_inputs_velocity_1_valid
  // linear_velocity_inputs_velocity_1_transmitted_safely
  // sleep_mode_input
  return true;
}

void
sick_safetyscanners2_interfaces__msg__ApplicationInputs__fini(sick_safetyscanners2_interfaces__msg__ApplicationInputs * msg)
{
  if (!msg) {
    return;
  }
  // unsafe_inputs_input_sources
  rosidl_runtime_c__boolean__Sequence__fini(&msg->unsafe_inputs_input_sources);
  // unsafe_inputs_flags
  rosidl_runtime_c__boolean__Sequence__fini(&msg->unsafe_inputs_flags);
  // monitoring_case_number_inputs
  rosidl_runtime_c__uint16__Sequence__fini(&msg->monitoring_case_number_inputs);
  // monitoring_case_number_inputs_flags
  rosidl_runtime_c__boolean__Sequence__fini(&msg->monitoring_case_number_inputs_flags);
  // linear_velocity_inputs_velocity_0
  // linear_velocity_inputs_velocity_0_valid
  // linear_velocity_inputs_velocity_0_transmitted_safely
  // linear_velocity_inputs_velocity_1
  // linear_velocity_inputs_velocity_1_valid
  // linear_velocity_inputs_velocity_1_transmitted_safely
  // sleep_mode_input
}

bool
sick_safetyscanners2_interfaces__msg__ApplicationInputs__are_equal(const sick_safetyscanners2_interfaces__msg__ApplicationInputs * lhs, const sick_safetyscanners2_interfaces__msg__ApplicationInputs * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // unsafe_inputs_input_sources
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->unsafe_inputs_input_sources), &(rhs->unsafe_inputs_input_sources)))
  {
    return false;
  }
  // unsafe_inputs_flags
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->unsafe_inputs_flags), &(rhs->unsafe_inputs_flags)))
  {
    return false;
  }
  // monitoring_case_number_inputs
  if (!rosidl_runtime_c__uint16__Sequence__are_equal(
      &(lhs->monitoring_case_number_inputs), &(rhs->monitoring_case_number_inputs)))
  {
    return false;
  }
  // monitoring_case_number_inputs_flags
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->monitoring_case_number_inputs_flags), &(rhs->monitoring_case_number_inputs_flags)))
  {
    return false;
  }
  // linear_velocity_inputs_velocity_0
  if (lhs->linear_velocity_inputs_velocity_0 != rhs->linear_velocity_inputs_velocity_0) {
    return false;
  }
  // linear_velocity_inputs_velocity_0_valid
  if (lhs->linear_velocity_inputs_velocity_0_valid != rhs->linear_velocity_inputs_velocity_0_valid) {
    return false;
  }
  // linear_velocity_inputs_velocity_0_transmitted_safely
  if (lhs->linear_velocity_inputs_velocity_0_transmitted_safely != rhs->linear_velocity_inputs_velocity_0_transmitted_safely) {
    return false;
  }
  // linear_velocity_inputs_velocity_1
  if (lhs->linear_velocity_inputs_velocity_1 != rhs->linear_velocity_inputs_velocity_1) {
    return false;
  }
  // linear_velocity_inputs_velocity_1_valid
  if (lhs->linear_velocity_inputs_velocity_1_valid != rhs->linear_velocity_inputs_velocity_1_valid) {
    return false;
  }
  // linear_velocity_inputs_velocity_1_transmitted_safely
  if (lhs->linear_velocity_inputs_velocity_1_transmitted_safely != rhs->linear_velocity_inputs_velocity_1_transmitted_safely) {
    return false;
  }
  // sleep_mode_input
  if (lhs->sleep_mode_input != rhs->sleep_mode_input) {
    return false;
  }
  return true;
}

bool
sick_safetyscanners2_interfaces__msg__ApplicationInputs__copy(
  const sick_safetyscanners2_interfaces__msg__ApplicationInputs * input,
  sick_safetyscanners2_interfaces__msg__ApplicationInputs * output)
{
  if (!input || !output) {
    return false;
  }
  // unsafe_inputs_input_sources
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->unsafe_inputs_input_sources), &(output->unsafe_inputs_input_sources)))
  {
    return false;
  }
  // unsafe_inputs_flags
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->unsafe_inputs_flags), &(output->unsafe_inputs_flags)))
  {
    return false;
  }
  // monitoring_case_number_inputs
  if (!rosidl_runtime_c__uint16__Sequence__copy(
      &(input->monitoring_case_number_inputs), &(output->monitoring_case_number_inputs)))
  {
    return false;
  }
  // monitoring_case_number_inputs_flags
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->monitoring_case_number_inputs_flags), &(output->monitoring_case_number_inputs_flags)))
  {
    return false;
  }
  // linear_velocity_inputs_velocity_0
  output->linear_velocity_inputs_velocity_0 = input->linear_velocity_inputs_velocity_0;
  // linear_velocity_inputs_velocity_0_valid
  output->linear_velocity_inputs_velocity_0_valid = input->linear_velocity_inputs_velocity_0_valid;
  // linear_velocity_inputs_velocity_0_transmitted_safely
  output->linear_velocity_inputs_velocity_0_transmitted_safely = input->linear_velocity_inputs_velocity_0_transmitted_safely;
  // linear_velocity_inputs_velocity_1
  output->linear_velocity_inputs_velocity_1 = input->linear_velocity_inputs_velocity_1;
  // linear_velocity_inputs_velocity_1_valid
  output->linear_velocity_inputs_velocity_1_valid = input->linear_velocity_inputs_velocity_1_valid;
  // linear_velocity_inputs_velocity_1_transmitted_safely
  output->linear_velocity_inputs_velocity_1_transmitted_safely = input->linear_velocity_inputs_velocity_1_transmitted_safely;
  // sleep_mode_input
  output->sleep_mode_input = input->sleep_mode_input;
  return true;
}

sick_safetyscanners2_interfaces__msg__ApplicationInputs *
sick_safetyscanners2_interfaces__msg__ApplicationInputs__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__ApplicationInputs * msg = (sick_safetyscanners2_interfaces__msg__ApplicationInputs *)allocator.allocate(sizeof(sick_safetyscanners2_interfaces__msg__ApplicationInputs), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sick_safetyscanners2_interfaces__msg__ApplicationInputs));
  bool success = sick_safetyscanners2_interfaces__msg__ApplicationInputs__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sick_safetyscanners2_interfaces__msg__ApplicationInputs__destroy(sick_safetyscanners2_interfaces__msg__ApplicationInputs * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sick_safetyscanners2_interfaces__msg__ApplicationInputs__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sick_safetyscanners2_interfaces__msg__ApplicationInputs__Sequence__init(sick_safetyscanners2_interfaces__msg__ApplicationInputs__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__ApplicationInputs * data = NULL;

  if (size) {
    data = (sick_safetyscanners2_interfaces__msg__ApplicationInputs *)allocator.zero_allocate(size, sizeof(sick_safetyscanners2_interfaces__msg__ApplicationInputs), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sick_safetyscanners2_interfaces__msg__ApplicationInputs__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sick_safetyscanners2_interfaces__msg__ApplicationInputs__fini(&data[i - 1]);
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
sick_safetyscanners2_interfaces__msg__ApplicationInputs__Sequence__fini(sick_safetyscanners2_interfaces__msg__ApplicationInputs__Sequence * array)
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
      sick_safetyscanners2_interfaces__msg__ApplicationInputs__fini(&array->data[i]);
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

sick_safetyscanners2_interfaces__msg__ApplicationInputs__Sequence *
sick_safetyscanners2_interfaces__msg__ApplicationInputs__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__ApplicationInputs__Sequence * array = (sick_safetyscanners2_interfaces__msg__ApplicationInputs__Sequence *)allocator.allocate(sizeof(sick_safetyscanners2_interfaces__msg__ApplicationInputs__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sick_safetyscanners2_interfaces__msg__ApplicationInputs__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sick_safetyscanners2_interfaces__msg__ApplicationInputs__Sequence__destroy(sick_safetyscanners2_interfaces__msg__ApplicationInputs__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sick_safetyscanners2_interfaces__msg__ApplicationInputs__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sick_safetyscanners2_interfaces__msg__ApplicationInputs__Sequence__are_equal(const sick_safetyscanners2_interfaces__msg__ApplicationInputs__Sequence * lhs, const sick_safetyscanners2_interfaces__msg__ApplicationInputs__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sick_safetyscanners2_interfaces__msg__ApplicationInputs__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sick_safetyscanners2_interfaces__msg__ApplicationInputs__Sequence__copy(
  const sick_safetyscanners2_interfaces__msg__ApplicationInputs__Sequence * input,
  sick_safetyscanners2_interfaces__msg__ApplicationInputs__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sick_safetyscanners2_interfaces__msg__ApplicationInputs);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sick_safetyscanners2_interfaces__msg__ApplicationInputs * data =
      (sick_safetyscanners2_interfaces__msg__ApplicationInputs *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sick_safetyscanners2_interfaces__msg__ApplicationInputs__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sick_safetyscanners2_interfaces__msg__ApplicationInputs__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sick_safetyscanners2_interfaces__msg__ApplicationInputs__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
