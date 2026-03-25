// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from sick_safetyscanners2_interfaces:msg/ApplicationOutputs.idl
// generated code does not contain a copyright notice
#include "sick_safetyscanners2_interfaces/msg/detail/application_outputs__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `evaluation_path_outputs_eval_out`
// Member `evaluation_path_outputs_is_safe`
// Member `evaluation_path_outputs_is_valid`
// Member `monitoring_case_number_outputs`
// Member `monitoring_case_number_outputs_flags`
// Member `resulting_velocity`
// Member `resulting_velocity_flags`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
sick_safetyscanners2_interfaces__msg__ApplicationOutputs__init(sick_safetyscanners2_interfaces__msg__ApplicationOutputs * msg)
{
  if (!msg) {
    return false;
  }
  // evaluation_path_outputs_eval_out
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->evaluation_path_outputs_eval_out, 0)) {
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__fini(msg);
    return false;
  }
  // evaluation_path_outputs_is_safe
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->evaluation_path_outputs_is_safe, 0)) {
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__fini(msg);
    return false;
  }
  // evaluation_path_outputs_is_valid
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->evaluation_path_outputs_is_valid, 0)) {
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__fini(msg);
    return false;
  }
  // monitoring_case_number_outputs
  if (!rosidl_runtime_c__uint16__Sequence__init(&msg->monitoring_case_number_outputs, 0)) {
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__fini(msg);
    return false;
  }
  // monitoring_case_number_outputs_flags
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->monitoring_case_number_outputs_flags, 0)) {
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__fini(msg);
    return false;
  }
  // sleep_mode_output
  // sleep_mode_output_valid
  // error_flag_contamination_warning
  // error_flag_contamination_error
  // error_flag_manipulation_error
  // error_flag_glare
  // error_flag_reference_contour_intruded
  // error_flag_critical_error
  // error_flags_are_valid
  // linear_velocity_outputs_velocity_0
  // linear_velocity_outputs_velocity_0_valid
  // linear_velocity_outputs_velocity_0_transmitted_safely
  // linear_velocity_outputs_velocity_1
  // linear_velocity_outputs_velocity_1_valid
  // linear_velocity_outputs_velocity_1_transmitted_safely
  // resulting_velocity
  if (!rosidl_runtime_c__int16__Sequence__init(&msg->resulting_velocity, 0)) {
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__fini(msg);
    return false;
  }
  // resulting_velocity_flags
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->resulting_velocity_flags, 0)) {
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__fini(msg);
    return false;
  }
  return true;
}

void
sick_safetyscanners2_interfaces__msg__ApplicationOutputs__fini(sick_safetyscanners2_interfaces__msg__ApplicationOutputs * msg)
{
  if (!msg) {
    return;
  }
  // evaluation_path_outputs_eval_out
  rosidl_runtime_c__boolean__Sequence__fini(&msg->evaluation_path_outputs_eval_out);
  // evaluation_path_outputs_is_safe
  rosidl_runtime_c__boolean__Sequence__fini(&msg->evaluation_path_outputs_is_safe);
  // evaluation_path_outputs_is_valid
  rosidl_runtime_c__boolean__Sequence__fini(&msg->evaluation_path_outputs_is_valid);
  // monitoring_case_number_outputs
  rosidl_runtime_c__uint16__Sequence__fini(&msg->monitoring_case_number_outputs);
  // monitoring_case_number_outputs_flags
  rosidl_runtime_c__boolean__Sequence__fini(&msg->monitoring_case_number_outputs_flags);
  // sleep_mode_output
  // sleep_mode_output_valid
  // error_flag_contamination_warning
  // error_flag_contamination_error
  // error_flag_manipulation_error
  // error_flag_glare
  // error_flag_reference_contour_intruded
  // error_flag_critical_error
  // error_flags_are_valid
  // linear_velocity_outputs_velocity_0
  // linear_velocity_outputs_velocity_0_valid
  // linear_velocity_outputs_velocity_0_transmitted_safely
  // linear_velocity_outputs_velocity_1
  // linear_velocity_outputs_velocity_1_valid
  // linear_velocity_outputs_velocity_1_transmitted_safely
  // resulting_velocity
  rosidl_runtime_c__int16__Sequence__fini(&msg->resulting_velocity);
  // resulting_velocity_flags
  rosidl_runtime_c__boolean__Sequence__fini(&msg->resulting_velocity_flags);
}

bool
sick_safetyscanners2_interfaces__msg__ApplicationOutputs__are_equal(const sick_safetyscanners2_interfaces__msg__ApplicationOutputs * lhs, const sick_safetyscanners2_interfaces__msg__ApplicationOutputs * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // evaluation_path_outputs_eval_out
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->evaluation_path_outputs_eval_out), &(rhs->evaluation_path_outputs_eval_out)))
  {
    return false;
  }
  // evaluation_path_outputs_is_safe
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->evaluation_path_outputs_is_safe), &(rhs->evaluation_path_outputs_is_safe)))
  {
    return false;
  }
  // evaluation_path_outputs_is_valid
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->evaluation_path_outputs_is_valid), &(rhs->evaluation_path_outputs_is_valid)))
  {
    return false;
  }
  // monitoring_case_number_outputs
  if (!rosidl_runtime_c__uint16__Sequence__are_equal(
      &(lhs->monitoring_case_number_outputs), &(rhs->monitoring_case_number_outputs)))
  {
    return false;
  }
  // monitoring_case_number_outputs_flags
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->monitoring_case_number_outputs_flags), &(rhs->monitoring_case_number_outputs_flags)))
  {
    return false;
  }
  // sleep_mode_output
  if (lhs->sleep_mode_output != rhs->sleep_mode_output) {
    return false;
  }
  // sleep_mode_output_valid
  if (lhs->sleep_mode_output_valid != rhs->sleep_mode_output_valid) {
    return false;
  }
  // error_flag_contamination_warning
  if (lhs->error_flag_contamination_warning != rhs->error_flag_contamination_warning) {
    return false;
  }
  // error_flag_contamination_error
  if (lhs->error_flag_contamination_error != rhs->error_flag_contamination_error) {
    return false;
  }
  // error_flag_manipulation_error
  if (lhs->error_flag_manipulation_error != rhs->error_flag_manipulation_error) {
    return false;
  }
  // error_flag_glare
  if (lhs->error_flag_glare != rhs->error_flag_glare) {
    return false;
  }
  // error_flag_reference_contour_intruded
  if (lhs->error_flag_reference_contour_intruded != rhs->error_flag_reference_contour_intruded) {
    return false;
  }
  // error_flag_critical_error
  if (lhs->error_flag_critical_error != rhs->error_flag_critical_error) {
    return false;
  }
  // error_flags_are_valid
  if (lhs->error_flags_are_valid != rhs->error_flags_are_valid) {
    return false;
  }
  // linear_velocity_outputs_velocity_0
  if (lhs->linear_velocity_outputs_velocity_0 != rhs->linear_velocity_outputs_velocity_0) {
    return false;
  }
  // linear_velocity_outputs_velocity_0_valid
  if (lhs->linear_velocity_outputs_velocity_0_valid != rhs->linear_velocity_outputs_velocity_0_valid) {
    return false;
  }
  // linear_velocity_outputs_velocity_0_transmitted_safely
  if (lhs->linear_velocity_outputs_velocity_0_transmitted_safely != rhs->linear_velocity_outputs_velocity_0_transmitted_safely) {
    return false;
  }
  // linear_velocity_outputs_velocity_1
  if (lhs->linear_velocity_outputs_velocity_1 != rhs->linear_velocity_outputs_velocity_1) {
    return false;
  }
  // linear_velocity_outputs_velocity_1_valid
  if (lhs->linear_velocity_outputs_velocity_1_valid != rhs->linear_velocity_outputs_velocity_1_valid) {
    return false;
  }
  // linear_velocity_outputs_velocity_1_transmitted_safely
  if (lhs->linear_velocity_outputs_velocity_1_transmitted_safely != rhs->linear_velocity_outputs_velocity_1_transmitted_safely) {
    return false;
  }
  // resulting_velocity
  if (!rosidl_runtime_c__int16__Sequence__are_equal(
      &(lhs->resulting_velocity), &(rhs->resulting_velocity)))
  {
    return false;
  }
  // resulting_velocity_flags
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->resulting_velocity_flags), &(rhs->resulting_velocity_flags)))
  {
    return false;
  }
  return true;
}

bool
sick_safetyscanners2_interfaces__msg__ApplicationOutputs__copy(
  const sick_safetyscanners2_interfaces__msg__ApplicationOutputs * input,
  sick_safetyscanners2_interfaces__msg__ApplicationOutputs * output)
{
  if (!input || !output) {
    return false;
  }
  // evaluation_path_outputs_eval_out
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->evaluation_path_outputs_eval_out), &(output->evaluation_path_outputs_eval_out)))
  {
    return false;
  }
  // evaluation_path_outputs_is_safe
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->evaluation_path_outputs_is_safe), &(output->evaluation_path_outputs_is_safe)))
  {
    return false;
  }
  // evaluation_path_outputs_is_valid
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->evaluation_path_outputs_is_valid), &(output->evaluation_path_outputs_is_valid)))
  {
    return false;
  }
  // monitoring_case_number_outputs
  if (!rosidl_runtime_c__uint16__Sequence__copy(
      &(input->monitoring_case_number_outputs), &(output->monitoring_case_number_outputs)))
  {
    return false;
  }
  // monitoring_case_number_outputs_flags
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->monitoring_case_number_outputs_flags), &(output->monitoring_case_number_outputs_flags)))
  {
    return false;
  }
  // sleep_mode_output
  output->sleep_mode_output = input->sleep_mode_output;
  // sleep_mode_output_valid
  output->sleep_mode_output_valid = input->sleep_mode_output_valid;
  // error_flag_contamination_warning
  output->error_flag_contamination_warning = input->error_flag_contamination_warning;
  // error_flag_contamination_error
  output->error_flag_contamination_error = input->error_flag_contamination_error;
  // error_flag_manipulation_error
  output->error_flag_manipulation_error = input->error_flag_manipulation_error;
  // error_flag_glare
  output->error_flag_glare = input->error_flag_glare;
  // error_flag_reference_contour_intruded
  output->error_flag_reference_contour_intruded = input->error_flag_reference_contour_intruded;
  // error_flag_critical_error
  output->error_flag_critical_error = input->error_flag_critical_error;
  // error_flags_are_valid
  output->error_flags_are_valid = input->error_flags_are_valid;
  // linear_velocity_outputs_velocity_0
  output->linear_velocity_outputs_velocity_0 = input->linear_velocity_outputs_velocity_0;
  // linear_velocity_outputs_velocity_0_valid
  output->linear_velocity_outputs_velocity_0_valid = input->linear_velocity_outputs_velocity_0_valid;
  // linear_velocity_outputs_velocity_0_transmitted_safely
  output->linear_velocity_outputs_velocity_0_transmitted_safely = input->linear_velocity_outputs_velocity_0_transmitted_safely;
  // linear_velocity_outputs_velocity_1
  output->linear_velocity_outputs_velocity_1 = input->linear_velocity_outputs_velocity_1;
  // linear_velocity_outputs_velocity_1_valid
  output->linear_velocity_outputs_velocity_1_valid = input->linear_velocity_outputs_velocity_1_valid;
  // linear_velocity_outputs_velocity_1_transmitted_safely
  output->linear_velocity_outputs_velocity_1_transmitted_safely = input->linear_velocity_outputs_velocity_1_transmitted_safely;
  // resulting_velocity
  if (!rosidl_runtime_c__int16__Sequence__copy(
      &(input->resulting_velocity), &(output->resulting_velocity)))
  {
    return false;
  }
  // resulting_velocity_flags
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->resulting_velocity_flags), &(output->resulting_velocity_flags)))
  {
    return false;
  }
  return true;
}

sick_safetyscanners2_interfaces__msg__ApplicationOutputs *
sick_safetyscanners2_interfaces__msg__ApplicationOutputs__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__ApplicationOutputs * msg = (sick_safetyscanners2_interfaces__msg__ApplicationOutputs *)allocator.allocate(sizeof(sick_safetyscanners2_interfaces__msg__ApplicationOutputs), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sick_safetyscanners2_interfaces__msg__ApplicationOutputs));
  bool success = sick_safetyscanners2_interfaces__msg__ApplicationOutputs__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sick_safetyscanners2_interfaces__msg__ApplicationOutputs__destroy(sick_safetyscanners2_interfaces__msg__ApplicationOutputs * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sick_safetyscanners2_interfaces__msg__ApplicationOutputs__Sequence__init(sick_safetyscanners2_interfaces__msg__ApplicationOutputs__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__ApplicationOutputs * data = NULL;

  if (size) {
    data = (sick_safetyscanners2_interfaces__msg__ApplicationOutputs *)allocator.zero_allocate(size, sizeof(sick_safetyscanners2_interfaces__msg__ApplicationOutputs), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sick_safetyscanners2_interfaces__msg__ApplicationOutputs__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sick_safetyscanners2_interfaces__msg__ApplicationOutputs__fini(&data[i - 1]);
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
sick_safetyscanners2_interfaces__msg__ApplicationOutputs__Sequence__fini(sick_safetyscanners2_interfaces__msg__ApplicationOutputs__Sequence * array)
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
      sick_safetyscanners2_interfaces__msg__ApplicationOutputs__fini(&array->data[i]);
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

sick_safetyscanners2_interfaces__msg__ApplicationOutputs__Sequence *
sick_safetyscanners2_interfaces__msg__ApplicationOutputs__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__ApplicationOutputs__Sequence * array = (sick_safetyscanners2_interfaces__msg__ApplicationOutputs__Sequence *)allocator.allocate(sizeof(sick_safetyscanners2_interfaces__msg__ApplicationOutputs__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sick_safetyscanners2_interfaces__msg__ApplicationOutputs__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sick_safetyscanners2_interfaces__msg__ApplicationOutputs__Sequence__destroy(sick_safetyscanners2_interfaces__msg__ApplicationOutputs__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sick_safetyscanners2_interfaces__msg__ApplicationOutputs__Sequence__are_equal(const sick_safetyscanners2_interfaces__msg__ApplicationOutputs__Sequence * lhs, const sick_safetyscanners2_interfaces__msg__ApplicationOutputs__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sick_safetyscanners2_interfaces__msg__ApplicationOutputs__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sick_safetyscanners2_interfaces__msg__ApplicationOutputs__Sequence__copy(
  const sick_safetyscanners2_interfaces__msg__ApplicationOutputs__Sequence * input,
  sick_safetyscanners2_interfaces__msg__ApplicationOutputs__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sick_safetyscanners2_interfaces__msg__ApplicationOutputs);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs * data =
      (sick_safetyscanners2_interfaces__msg__ApplicationOutputs *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sick_safetyscanners2_interfaces__msg__ApplicationOutputs__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sick_safetyscanners2_interfaces__msg__ApplicationOutputs__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sick_safetyscanners2_interfaces__msg__ApplicationOutputs__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
