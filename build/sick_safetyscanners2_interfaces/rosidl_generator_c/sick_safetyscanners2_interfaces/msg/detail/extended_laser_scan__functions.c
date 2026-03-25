// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from sick_safetyscanners2_interfaces:msg/ExtendedLaserScan.idl
// generated code does not contain a copyright notice
#include "sick_safetyscanners2_interfaces/msg/detail/extended_laser_scan__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `laser_scan`
#include "sensor_msgs/msg/detail/laser_scan__functions.h"
// Member `reflektor_status`
// Member `reflektor_median`
// Member `intrusion`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__init(sick_safetyscanners2_interfaces__msg__ExtendedLaserScan * msg)
{
  if (!msg) {
    return false;
  }
  // laser_scan
  if (!sensor_msgs__msg__LaserScan__init(&msg->laser_scan)) {
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__fini(msg);
    return false;
  }
  // reflektor_status
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->reflektor_status, 0)) {
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__fini(msg);
    return false;
  }
  // reflektor_median
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->reflektor_median, 0)) {
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__fini(msg);
    return false;
  }
  // intrusion
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->intrusion, 0)) {
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__fini(msg);
    return false;
  }
  return true;
}

void
sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__fini(sick_safetyscanners2_interfaces__msg__ExtendedLaserScan * msg)
{
  if (!msg) {
    return;
  }
  // laser_scan
  sensor_msgs__msg__LaserScan__fini(&msg->laser_scan);
  // reflektor_status
  rosidl_runtime_c__boolean__Sequence__fini(&msg->reflektor_status);
  // reflektor_median
  rosidl_runtime_c__boolean__Sequence__fini(&msg->reflektor_median);
  // intrusion
  rosidl_runtime_c__boolean__Sequence__fini(&msg->intrusion);
}

bool
sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__are_equal(const sick_safetyscanners2_interfaces__msg__ExtendedLaserScan * lhs, const sick_safetyscanners2_interfaces__msg__ExtendedLaserScan * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // laser_scan
  if (!sensor_msgs__msg__LaserScan__are_equal(
      &(lhs->laser_scan), &(rhs->laser_scan)))
  {
    return false;
  }
  // reflektor_status
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->reflektor_status), &(rhs->reflektor_status)))
  {
    return false;
  }
  // reflektor_median
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->reflektor_median), &(rhs->reflektor_median)))
  {
    return false;
  }
  // intrusion
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->intrusion), &(rhs->intrusion)))
  {
    return false;
  }
  return true;
}

bool
sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__copy(
  const sick_safetyscanners2_interfaces__msg__ExtendedLaserScan * input,
  sick_safetyscanners2_interfaces__msg__ExtendedLaserScan * output)
{
  if (!input || !output) {
    return false;
  }
  // laser_scan
  if (!sensor_msgs__msg__LaserScan__copy(
      &(input->laser_scan), &(output->laser_scan)))
  {
    return false;
  }
  // reflektor_status
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->reflektor_status), &(output->reflektor_status)))
  {
    return false;
  }
  // reflektor_median
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->reflektor_median), &(output->reflektor_median)))
  {
    return false;
  }
  // intrusion
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->intrusion), &(output->intrusion)))
  {
    return false;
  }
  return true;
}

sick_safetyscanners2_interfaces__msg__ExtendedLaserScan *
sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__ExtendedLaserScan * msg = (sick_safetyscanners2_interfaces__msg__ExtendedLaserScan *)allocator.allocate(sizeof(sick_safetyscanners2_interfaces__msg__ExtendedLaserScan), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sick_safetyscanners2_interfaces__msg__ExtendedLaserScan));
  bool success = sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__destroy(sick_safetyscanners2_interfaces__msg__ExtendedLaserScan * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__Sequence__init(sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__ExtendedLaserScan * data = NULL;

  if (size) {
    data = (sick_safetyscanners2_interfaces__msg__ExtendedLaserScan *)allocator.zero_allocate(size, sizeof(sick_safetyscanners2_interfaces__msg__ExtendedLaserScan), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__fini(&data[i - 1]);
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
sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__Sequence__fini(sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__Sequence * array)
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
      sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__fini(&array->data[i]);
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

sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__Sequence *
sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__Sequence * array = (sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__Sequence *)allocator.allocate(sizeof(sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__Sequence__destroy(sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__Sequence__are_equal(const sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__Sequence * lhs, const sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__Sequence__copy(
  const sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__Sequence * input,
  sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sick_safetyscanners2_interfaces__msg__ExtendedLaserScan);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan * data =
      (sick_safetyscanners2_interfaces__msg__ExtendedLaserScan *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
