// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from sick_safetyscanners2_interfaces:msg/GeneralSystemState.idl
// generated code does not contain a copyright notice
#include "sick_safetyscanners2_interfaces/msg/detail/general_system_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `safe_cut_off_path`
// Member `non_safe_cut_off_path`
// Member `reset_required_cut_off_path`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
sick_safetyscanners2_interfaces__msg__GeneralSystemState__init(sick_safetyscanners2_interfaces__msg__GeneralSystemState * msg)
{
  if (!msg) {
    return false;
  }
  // run_mode_active
  // standby_mode_active
  // contamination_warning
  // contamination_error
  // reference_contour_status
  // manipulation_status
  // safe_cut_off_path
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->safe_cut_off_path, 0)) {
    sick_safetyscanners2_interfaces__msg__GeneralSystemState__fini(msg);
    return false;
  }
  // non_safe_cut_off_path
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->non_safe_cut_off_path, 0)) {
    sick_safetyscanners2_interfaces__msg__GeneralSystemState__fini(msg);
    return false;
  }
  // reset_required_cut_off_path
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->reset_required_cut_off_path, 0)) {
    sick_safetyscanners2_interfaces__msg__GeneralSystemState__fini(msg);
    return false;
  }
  // current_monitoring_case_no_table_1
  // current_monitoring_case_no_table_2
  // current_monitoring_case_no_table_3
  // current_monitoring_case_no_table_4
  // application_error
  // device_error
  return true;
}

void
sick_safetyscanners2_interfaces__msg__GeneralSystemState__fini(sick_safetyscanners2_interfaces__msg__GeneralSystemState * msg)
{
  if (!msg) {
    return;
  }
  // run_mode_active
  // standby_mode_active
  // contamination_warning
  // contamination_error
  // reference_contour_status
  // manipulation_status
  // safe_cut_off_path
  rosidl_runtime_c__boolean__Sequence__fini(&msg->safe_cut_off_path);
  // non_safe_cut_off_path
  rosidl_runtime_c__boolean__Sequence__fini(&msg->non_safe_cut_off_path);
  // reset_required_cut_off_path
  rosidl_runtime_c__boolean__Sequence__fini(&msg->reset_required_cut_off_path);
  // current_monitoring_case_no_table_1
  // current_monitoring_case_no_table_2
  // current_monitoring_case_no_table_3
  // current_monitoring_case_no_table_4
  // application_error
  // device_error
}

bool
sick_safetyscanners2_interfaces__msg__GeneralSystemState__are_equal(const sick_safetyscanners2_interfaces__msg__GeneralSystemState * lhs, const sick_safetyscanners2_interfaces__msg__GeneralSystemState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // run_mode_active
  if (lhs->run_mode_active != rhs->run_mode_active) {
    return false;
  }
  // standby_mode_active
  if (lhs->standby_mode_active != rhs->standby_mode_active) {
    return false;
  }
  // contamination_warning
  if (lhs->contamination_warning != rhs->contamination_warning) {
    return false;
  }
  // contamination_error
  if (lhs->contamination_error != rhs->contamination_error) {
    return false;
  }
  // reference_contour_status
  if (lhs->reference_contour_status != rhs->reference_contour_status) {
    return false;
  }
  // manipulation_status
  if (lhs->manipulation_status != rhs->manipulation_status) {
    return false;
  }
  // safe_cut_off_path
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->safe_cut_off_path), &(rhs->safe_cut_off_path)))
  {
    return false;
  }
  // non_safe_cut_off_path
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->non_safe_cut_off_path), &(rhs->non_safe_cut_off_path)))
  {
    return false;
  }
  // reset_required_cut_off_path
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->reset_required_cut_off_path), &(rhs->reset_required_cut_off_path)))
  {
    return false;
  }
  // current_monitoring_case_no_table_1
  if (lhs->current_monitoring_case_no_table_1 != rhs->current_monitoring_case_no_table_1) {
    return false;
  }
  // current_monitoring_case_no_table_2
  if (lhs->current_monitoring_case_no_table_2 != rhs->current_monitoring_case_no_table_2) {
    return false;
  }
  // current_monitoring_case_no_table_3
  if (lhs->current_monitoring_case_no_table_3 != rhs->current_monitoring_case_no_table_3) {
    return false;
  }
  // current_monitoring_case_no_table_4
  if (lhs->current_monitoring_case_no_table_4 != rhs->current_monitoring_case_no_table_4) {
    return false;
  }
  // application_error
  if (lhs->application_error != rhs->application_error) {
    return false;
  }
  // device_error
  if (lhs->device_error != rhs->device_error) {
    return false;
  }
  return true;
}

bool
sick_safetyscanners2_interfaces__msg__GeneralSystemState__copy(
  const sick_safetyscanners2_interfaces__msg__GeneralSystemState * input,
  sick_safetyscanners2_interfaces__msg__GeneralSystemState * output)
{
  if (!input || !output) {
    return false;
  }
  // run_mode_active
  output->run_mode_active = input->run_mode_active;
  // standby_mode_active
  output->standby_mode_active = input->standby_mode_active;
  // contamination_warning
  output->contamination_warning = input->contamination_warning;
  // contamination_error
  output->contamination_error = input->contamination_error;
  // reference_contour_status
  output->reference_contour_status = input->reference_contour_status;
  // manipulation_status
  output->manipulation_status = input->manipulation_status;
  // safe_cut_off_path
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->safe_cut_off_path), &(output->safe_cut_off_path)))
  {
    return false;
  }
  // non_safe_cut_off_path
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->non_safe_cut_off_path), &(output->non_safe_cut_off_path)))
  {
    return false;
  }
  // reset_required_cut_off_path
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->reset_required_cut_off_path), &(output->reset_required_cut_off_path)))
  {
    return false;
  }
  // current_monitoring_case_no_table_1
  output->current_monitoring_case_no_table_1 = input->current_monitoring_case_no_table_1;
  // current_monitoring_case_no_table_2
  output->current_monitoring_case_no_table_2 = input->current_monitoring_case_no_table_2;
  // current_monitoring_case_no_table_3
  output->current_monitoring_case_no_table_3 = input->current_monitoring_case_no_table_3;
  // current_monitoring_case_no_table_4
  output->current_monitoring_case_no_table_4 = input->current_monitoring_case_no_table_4;
  // application_error
  output->application_error = input->application_error;
  // device_error
  output->device_error = input->device_error;
  return true;
}

sick_safetyscanners2_interfaces__msg__GeneralSystemState *
sick_safetyscanners2_interfaces__msg__GeneralSystemState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__GeneralSystemState * msg = (sick_safetyscanners2_interfaces__msg__GeneralSystemState *)allocator.allocate(sizeof(sick_safetyscanners2_interfaces__msg__GeneralSystemState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sick_safetyscanners2_interfaces__msg__GeneralSystemState));
  bool success = sick_safetyscanners2_interfaces__msg__GeneralSystemState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sick_safetyscanners2_interfaces__msg__GeneralSystemState__destroy(sick_safetyscanners2_interfaces__msg__GeneralSystemState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sick_safetyscanners2_interfaces__msg__GeneralSystemState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sick_safetyscanners2_interfaces__msg__GeneralSystemState__Sequence__init(sick_safetyscanners2_interfaces__msg__GeneralSystemState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__GeneralSystemState * data = NULL;

  if (size) {
    data = (sick_safetyscanners2_interfaces__msg__GeneralSystemState *)allocator.zero_allocate(size, sizeof(sick_safetyscanners2_interfaces__msg__GeneralSystemState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sick_safetyscanners2_interfaces__msg__GeneralSystemState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sick_safetyscanners2_interfaces__msg__GeneralSystemState__fini(&data[i - 1]);
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
sick_safetyscanners2_interfaces__msg__GeneralSystemState__Sequence__fini(sick_safetyscanners2_interfaces__msg__GeneralSystemState__Sequence * array)
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
      sick_safetyscanners2_interfaces__msg__GeneralSystemState__fini(&array->data[i]);
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

sick_safetyscanners2_interfaces__msg__GeneralSystemState__Sequence *
sick_safetyscanners2_interfaces__msg__GeneralSystemState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__GeneralSystemState__Sequence * array = (sick_safetyscanners2_interfaces__msg__GeneralSystemState__Sequence *)allocator.allocate(sizeof(sick_safetyscanners2_interfaces__msg__GeneralSystemState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sick_safetyscanners2_interfaces__msg__GeneralSystemState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sick_safetyscanners2_interfaces__msg__GeneralSystemState__Sequence__destroy(sick_safetyscanners2_interfaces__msg__GeneralSystemState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sick_safetyscanners2_interfaces__msg__GeneralSystemState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sick_safetyscanners2_interfaces__msg__GeneralSystemState__Sequence__are_equal(const sick_safetyscanners2_interfaces__msg__GeneralSystemState__Sequence * lhs, const sick_safetyscanners2_interfaces__msg__GeneralSystemState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sick_safetyscanners2_interfaces__msg__GeneralSystemState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sick_safetyscanners2_interfaces__msg__GeneralSystemState__Sequence__copy(
  const sick_safetyscanners2_interfaces__msg__GeneralSystemState__Sequence * input,
  sick_safetyscanners2_interfaces__msg__GeneralSystemState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sick_safetyscanners2_interfaces__msg__GeneralSystemState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sick_safetyscanners2_interfaces__msg__GeneralSystemState * data =
      (sick_safetyscanners2_interfaces__msg__GeneralSystemState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sick_safetyscanners2_interfaces__msg__GeneralSystemState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sick_safetyscanners2_interfaces__msg__GeneralSystemState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sick_safetyscanners2_interfaces__msg__GeneralSystemState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
