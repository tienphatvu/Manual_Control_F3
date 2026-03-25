// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from sick_safetyscanners2_interfaces:msg/DerivedValues.idl
// generated code does not contain a copyright notice
#include "sick_safetyscanners2_interfaces/msg/detail/derived_values__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
sick_safetyscanners2_interfaces__msg__DerivedValues__init(sick_safetyscanners2_interfaces__msg__DerivedValues * msg)
{
  if (!msg) {
    return false;
  }
  // multiplication_factor
  // number_of_beams
  // scan_time
  // start_angle
  // angular_beam_resolution
  // interbeam_period
  return true;
}

void
sick_safetyscanners2_interfaces__msg__DerivedValues__fini(sick_safetyscanners2_interfaces__msg__DerivedValues * msg)
{
  if (!msg) {
    return;
  }
  // multiplication_factor
  // number_of_beams
  // scan_time
  // start_angle
  // angular_beam_resolution
  // interbeam_period
}

bool
sick_safetyscanners2_interfaces__msg__DerivedValues__are_equal(const sick_safetyscanners2_interfaces__msg__DerivedValues * lhs, const sick_safetyscanners2_interfaces__msg__DerivedValues * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // multiplication_factor
  if (lhs->multiplication_factor != rhs->multiplication_factor) {
    return false;
  }
  // number_of_beams
  if (lhs->number_of_beams != rhs->number_of_beams) {
    return false;
  }
  // scan_time
  if (lhs->scan_time != rhs->scan_time) {
    return false;
  }
  // start_angle
  if (lhs->start_angle != rhs->start_angle) {
    return false;
  }
  // angular_beam_resolution
  if (lhs->angular_beam_resolution != rhs->angular_beam_resolution) {
    return false;
  }
  // interbeam_period
  if (lhs->interbeam_period != rhs->interbeam_period) {
    return false;
  }
  return true;
}

bool
sick_safetyscanners2_interfaces__msg__DerivedValues__copy(
  const sick_safetyscanners2_interfaces__msg__DerivedValues * input,
  sick_safetyscanners2_interfaces__msg__DerivedValues * output)
{
  if (!input || !output) {
    return false;
  }
  // multiplication_factor
  output->multiplication_factor = input->multiplication_factor;
  // number_of_beams
  output->number_of_beams = input->number_of_beams;
  // scan_time
  output->scan_time = input->scan_time;
  // start_angle
  output->start_angle = input->start_angle;
  // angular_beam_resolution
  output->angular_beam_resolution = input->angular_beam_resolution;
  // interbeam_period
  output->interbeam_period = input->interbeam_period;
  return true;
}

sick_safetyscanners2_interfaces__msg__DerivedValues *
sick_safetyscanners2_interfaces__msg__DerivedValues__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__DerivedValues * msg = (sick_safetyscanners2_interfaces__msg__DerivedValues *)allocator.allocate(sizeof(sick_safetyscanners2_interfaces__msg__DerivedValues), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sick_safetyscanners2_interfaces__msg__DerivedValues));
  bool success = sick_safetyscanners2_interfaces__msg__DerivedValues__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sick_safetyscanners2_interfaces__msg__DerivedValues__destroy(sick_safetyscanners2_interfaces__msg__DerivedValues * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sick_safetyscanners2_interfaces__msg__DerivedValues__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sick_safetyscanners2_interfaces__msg__DerivedValues__Sequence__init(sick_safetyscanners2_interfaces__msg__DerivedValues__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__DerivedValues * data = NULL;

  if (size) {
    data = (sick_safetyscanners2_interfaces__msg__DerivedValues *)allocator.zero_allocate(size, sizeof(sick_safetyscanners2_interfaces__msg__DerivedValues), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sick_safetyscanners2_interfaces__msg__DerivedValues__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sick_safetyscanners2_interfaces__msg__DerivedValues__fini(&data[i - 1]);
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
sick_safetyscanners2_interfaces__msg__DerivedValues__Sequence__fini(sick_safetyscanners2_interfaces__msg__DerivedValues__Sequence * array)
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
      sick_safetyscanners2_interfaces__msg__DerivedValues__fini(&array->data[i]);
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

sick_safetyscanners2_interfaces__msg__DerivedValues__Sequence *
sick_safetyscanners2_interfaces__msg__DerivedValues__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__DerivedValues__Sequence * array = (sick_safetyscanners2_interfaces__msg__DerivedValues__Sequence *)allocator.allocate(sizeof(sick_safetyscanners2_interfaces__msg__DerivedValues__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sick_safetyscanners2_interfaces__msg__DerivedValues__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sick_safetyscanners2_interfaces__msg__DerivedValues__Sequence__destroy(sick_safetyscanners2_interfaces__msg__DerivedValues__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sick_safetyscanners2_interfaces__msg__DerivedValues__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sick_safetyscanners2_interfaces__msg__DerivedValues__Sequence__are_equal(const sick_safetyscanners2_interfaces__msg__DerivedValues__Sequence * lhs, const sick_safetyscanners2_interfaces__msg__DerivedValues__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sick_safetyscanners2_interfaces__msg__DerivedValues__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sick_safetyscanners2_interfaces__msg__DerivedValues__Sequence__copy(
  const sick_safetyscanners2_interfaces__msg__DerivedValues__Sequence * input,
  sick_safetyscanners2_interfaces__msg__DerivedValues__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sick_safetyscanners2_interfaces__msg__DerivedValues);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sick_safetyscanners2_interfaces__msg__DerivedValues * data =
      (sick_safetyscanners2_interfaces__msg__DerivedValues *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sick_safetyscanners2_interfaces__msg__DerivedValues__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sick_safetyscanners2_interfaces__msg__DerivedValues__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sick_safetyscanners2_interfaces__msg__DerivedValues__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
