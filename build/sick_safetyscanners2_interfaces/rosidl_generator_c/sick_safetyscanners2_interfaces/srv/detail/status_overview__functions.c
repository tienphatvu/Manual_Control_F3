// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from sick_safetyscanners2_interfaces:srv/StatusOverview.idl
// generated code does not contain a copyright notice
#include "sick_safetyscanners2_interfaces/srv/detail/status_overview__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
sick_safetyscanners2_interfaces__srv__StatusOverview_Request__init(sick_safetyscanners2_interfaces__srv__StatusOverview_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
sick_safetyscanners2_interfaces__srv__StatusOverview_Request__fini(sick_safetyscanners2_interfaces__srv__StatusOverview_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
sick_safetyscanners2_interfaces__srv__StatusOverview_Request__are_equal(const sick_safetyscanners2_interfaces__srv__StatusOverview_Request * lhs, const sick_safetyscanners2_interfaces__srv__StatusOverview_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
sick_safetyscanners2_interfaces__srv__StatusOverview_Request__copy(
  const sick_safetyscanners2_interfaces__srv__StatusOverview_Request * input,
  sick_safetyscanners2_interfaces__srv__StatusOverview_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

sick_safetyscanners2_interfaces__srv__StatusOverview_Request *
sick_safetyscanners2_interfaces__srv__StatusOverview_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__srv__StatusOverview_Request * msg = (sick_safetyscanners2_interfaces__srv__StatusOverview_Request *)allocator.allocate(sizeof(sick_safetyscanners2_interfaces__srv__StatusOverview_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sick_safetyscanners2_interfaces__srv__StatusOverview_Request));
  bool success = sick_safetyscanners2_interfaces__srv__StatusOverview_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sick_safetyscanners2_interfaces__srv__StatusOverview_Request__destroy(sick_safetyscanners2_interfaces__srv__StatusOverview_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sick_safetyscanners2_interfaces__srv__StatusOverview_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sick_safetyscanners2_interfaces__srv__StatusOverview_Request__Sequence__init(sick_safetyscanners2_interfaces__srv__StatusOverview_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__srv__StatusOverview_Request * data = NULL;

  if (size) {
    data = (sick_safetyscanners2_interfaces__srv__StatusOverview_Request *)allocator.zero_allocate(size, sizeof(sick_safetyscanners2_interfaces__srv__StatusOverview_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sick_safetyscanners2_interfaces__srv__StatusOverview_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sick_safetyscanners2_interfaces__srv__StatusOverview_Request__fini(&data[i - 1]);
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
sick_safetyscanners2_interfaces__srv__StatusOverview_Request__Sequence__fini(sick_safetyscanners2_interfaces__srv__StatusOverview_Request__Sequence * array)
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
      sick_safetyscanners2_interfaces__srv__StatusOverview_Request__fini(&array->data[i]);
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

sick_safetyscanners2_interfaces__srv__StatusOverview_Request__Sequence *
sick_safetyscanners2_interfaces__srv__StatusOverview_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__srv__StatusOverview_Request__Sequence * array = (sick_safetyscanners2_interfaces__srv__StatusOverview_Request__Sequence *)allocator.allocate(sizeof(sick_safetyscanners2_interfaces__srv__StatusOverview_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sick_safetyscanners2_interfaces__srv__StatusOverview_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sick_safetyscanners2_interfaces__srv__StatusOverview_Request__Sequence__destroy(sick_safetyscanners2_interfaces__srv__StatusOverview_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sick_safetyscanners2_interfaces__srv__StatusOverview_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sick_safetyscanners2_interfaces__srv__StatusOverview_Request__Sequence__are_equal(const sick_safetyscanners2_interfaces__srv__StatusOverview_Request__Sequence * lhs, const sick_safetyscanners2_interfaces__srv__StatusOverview_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sick_safetyscanners2_interfaces__srv__StatusOverview_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sick_safetyscanners2_interfaces__srv__StatusOverview_Request__Sequence__copy(
  const sick_safetyscanners2_interfaces__srv__StatusOverview_Request__Sequence * input,
  sick_safetyscanners2_interfaces__srv__StatusOverview_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sick_safetyscanners2_interfaces__srv__StatusOverview_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sick_safetyscanners2_interfaces__srv__StatusOverview_Request * data =
      (sick_safetyscanners2_interfaces__srv__StatusOverview_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sick_safetyscanners2_interfaces__srv__StatusOverview_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sick_safetyscanners2_interfaces__srv__StatusOverview_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sick_safetyscanners2_interfaces__srv__StatusOverview_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `version_c_version`
// Member `current_time`
// Member `error_info_time`
#include "rosidl_runtime_c/string_functions.h"

bool
sick_safetyscanners2_interfaces__srv__StatusOverview_Response__init(sick_safetyscanners2_interfaces__srv__StatusOverview_Response * msg)
{
  if (!msg) {
    return false;
  }
  // version_c_version
  if (!rosidl_runtime_c__String__init(&msg->version_c_version)) {
    sick_safetyscanners2_interfaces__srv__StatusOverview_Response__fini(msg);
    return false;
  }
  // version_major_version_number
  // version_minor_version_number
  // version_release_number
  // device_state
  // config_state
  // application_state
  // current_time_power_on_count
  // current_time
  if (!rosidl_runtime_c__String__init(&msg->current_time)) {
    sick_safetyscanners2_interfaces__srv__StatusOverview_Response__fini(msg);
    return false;
  }
  // current_time_time
  // current_time_date
  // error_info_code
  // error_info_time
  if (!rosidl_runtime_c__String__init(&msg->error_info_time)) {
    sick_safetyscanners2_interfaces__srv__StatusOverview_Response__fini(msg);
    return false;
  }
  // error_info_time_time
  // error_info_time_date
  return true;
}

void
sick_safetyscanners2_interfaces__srv__StatusOverview_Response__fini(sick_safetyscanners2_interfaces__srv__StatusOverview_Response * msg)
{
  if (!msg) {
    return;
  }
  // version_c_version
  rosidl_runtime_c__String__fini(&msg->version_c_version);
  // version_major_version_number
  // version_minor_version_number
  // version_release_number
  // device_state
  // config_state
  // application_state
  // current_time_power_on_count
  // current_time
  rosidl_runtime_c__String__fini(&msg->current_time);
  // current_time_time
  // current_time_date
  // error_info_code
  // error_info_time
  rosidl_runtime_c__String__fini(&msg->error_info_time);
  // error_info_time_time
  // error_info_time_date
}

bool
sick_safetyscanners2_interfaces__srv__StatusOverview_Response__are_equal(const sick_safetyscanners2_interfaces__srv__StatusOverview_Response * lhs, const sick_safetyscanners2_interfaces__srv__StatusOverview_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // version_c_version
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->version_c_version), &(rhs->version_c_version)))
  {
    return false;
  }
  // version_major_version_number
  if (lhs->version_major_version_number != rhs->version_major_version_number) {
    return false;
  }
  // version_minor_version_number
  if (lhs->version_minor_version_number != rhs->version_minor_version_number) {
    return false;
  }
  // version_release_number
  if (lhs->version_release_number != rhs->version_release_number) {
    return false;
  }
  // device_state
  if (lhs->device_state != rhs->device_state) {
    return false;
  }
  // config_state
  if (lhs->config_state != rhs->config_state) {
    return false;
  }
  // application_state
  if (lhs->application_state != rhs->application_state) {
    return false;
  }
  // current_time_power_on_count
  if (lhs->current_time_power_on_count != rhs->current_time_power_on_count) {
    return false;
  }
  // current_time
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->current_time), &(rhs->current_time)))
  {
    return false;
  }
  // current_time_time
  if (lhs->current_time_time != rhs->current_time_time) {
    return false;
  }
  // current_time_date
  if (lhs->current_time_date != rhs->current_time_date) {
    return false;
  }
  // error_info_code
  if (lhs->error_info_code != rhs->error_info_code) {
    return false;
  }
  // error_info_time
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->error_info_time), &(rhs->error_info_time)))
  {
    return false;
  }
  // error_info_time_time
  if (lhs->error_info_time_time != rhs->error_info_time_time) {
    return false;
  }
  // error_info_time_date
  if (lhs->error_info_time_date != rhs->error_info_time_date) {
    return false;
  }
  return true;
}

bool
sick_safetyscanners2_interfaces__srv__StatusOverview_Response__copy(
  const sick_safetyscanners2_interfaces__srv__StatusOverview_Response * input,
  sick_safetyscanners2_interfaces__srv__StatusOverview_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // version_c_version
  if (!rosidl_runtime_c__String__copy(
      &(input->version_c_version), &(output->version_c_version)))
  {
    return false;
  }
  // version_major_version_number
  output->version_major_version_number = input->version_major_version_number;
  // version_minor_version_number
  output->version_minor_version_number = input->version_minor_version_number;
  // version_release_number
  output->version_release_number = input->version_release_number;
  // device_state
  output->device_state = input->device_state;
  // config_state
  output->config_state = input->config_state;
  // application_state
  output->application_state = input->application_state;
  // current_time_power_on_count
  output->current_time_power_on_count = input->current_time_power_on_count;
  // current_time
  if (!rosidl_runtime_c__String__copy(
      &(input->current_time), &(output->current_time)))
  {
    return false;
  }
  // current_time_time
  output->current_time_time = input->current_time_time;
  // current_time_date
  output->current_time_date = input->current_time_date;
  // error_info_code
  output->error_info_code = input->error_info_code;
  // error_info_time
  if (!rosidl_runtime_c__String__copy(
      &(input->error_info_time), &(output->error_info_time)))
  {
    return false;
  }
  // error_info_time_time
  output->error_info_time_time = input->error_info_time_time;
  // error_info_time_date
  output->error_info_time_date = input->error_info_time_date;
  return true;
}

sick_safetyscanners2_interfaces__srv__StatusOverview_Response *
sick_safetyscanners2_interfaces__srv__StatusOverview_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__srv__StatusOverview_Response * msg = (sick_safetyscanners2_interfaces__srv__StatusOverview_Response *)allocator.allocate(sizeof(sick_safetyscanners2_interfaces__srv__StatusOverview_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sick_safetyscanners2_interfaces__srv__StatusOverview_Response));
  bool success = sick_safetyscanners2_interfaces__srv__StatusOverview_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sick_safetyscanners2_interfaces__srv__StatusOverview_Response__destroy(sick_safetyscanners2_interfaces__srv__StatusOverview_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sick_safetyscanners2_interfaces__srv__StatusOverview_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sick_safetyscanners2_interfaces__srv__StatusOverview_Response__Sequence__init(sick_safetyscanners2_interfaces__srv__StatusOverview_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__srv__StatusOverview_Response * data = NULL;

  if (size) {
    data = (sick_safetyscanners2_interfaces__srv__StatusOverview_Response *)allocator.zero_allocate(size, sizeof(sick_safetyscanners2_interfaces__srv__StatusOverview_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sick_safetyscanners2_interfaces__srv__StatusOverview_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sick_safetyscanners2_interfaces__srv__StatusOverview_Response__fini(&data[i - 1]);
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
sick_safetyscanners2_interfaces__srv__StatusOverview_Response__Sequence__fini(sick_safetyscanners2_interfaces__srv__StatusOverview_Response__Sequence * array)
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
      sick_safetyscanners2_interfaces__srv__StatusOverview_Response__fini(&array->data[i]);
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

sick_safetyscanners2_interfaces__srv__StatusOverview_Response__Sequence *
sick_safetyscanners2_interfaces__srv__StatusOverview_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__srv__StatusOverview_Response__Sequence * array = (sick_safetyscanners2_interfaces__srv__StatusOverview_Response__Sequence *)allocator.allocate(sizeof(sick_safetyscanners2_interfaces__srv__StatusOverview_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sick_safetyscanners2_interfaces__srv__StatusOverview_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sick_safetyscanners2_interfaces__srv__StatusOverview_Response__Sequence__destroy(sick_safetyscanners2_interfaces__srv__StatusOverview_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sick_safetyscanners2_interfaces__srv__StatusOverview_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sick_safetyscanners2_interfaces__srv__StatusOverview_Response__Sequence__are_equal(const sick_safetyscanners2_interfaces__srv__StatusOverview_Response__Sequence * lhs, const sick_safetyscanners2_interfaces__srv__StatusOverview_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sick_safetyscanners2_interfaces__srv__StatusOverview_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sick_safetyscanners2_interfaces__srv__StatusOverview_Response__Sequence__copy(
  const sick_safetyscanners2_interfaces__srv__StatusOverview_Response__Sequence * input,
  sick_safetyscanners2_interfaces__srv__StatusOverview_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sick_safetyscanners2_interfaces__srv__StatusOverview_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sick_safetyscanners2_interfaces__srv__StatusOverview_Response * data =
      (sick_safetyscanners2_interfaces__srv__StatusOverview_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sick_safetyscanners2_interfaces__srv__StatusOverview_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sick_safetyscanners2_interfaces__srv__StatusOverview_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sick_safetyscanners2_interfaces__srv__StatusOverview_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
