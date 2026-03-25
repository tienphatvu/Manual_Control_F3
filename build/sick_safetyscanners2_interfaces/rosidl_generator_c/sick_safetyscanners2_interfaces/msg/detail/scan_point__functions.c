// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from sick_safetyscanners2_interfaces:msg/ScanPoint.idl
// generated code does not contain a copyright notice
#include "sick_safetyscanners2_interfaces/msg/detail/scan_point__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
sick_safetyscanners2_interfaces__msg__ScanPoint__init(sick_safetyscanners2_interfaces__msg__ScanPoint * msg)
{
  if (!msg) {
    return false;
  }
  // angle
  // distance
  // reflectivity
  // valid
  // infinite
  // glare
  // reflector
  // contamination
  // contamination_warning
  return true;
}

void
sick_safetyscanners2_interfaces__msg__ScanPoint__fini(sick_safetyscanners2_interfaces__msg__ScanPoint * msg)
{
  if (!msg) {
    return;
  }
  // angle
  // distance
  // reflectivity
  // valid
  // infinite
  // glare
  // reflector
  // contamination
  // contamination_warning
}

bool
sick_safetyscanners2_interfaces__msg__ScanPoint__are_equal(const sick_safetyscanners2_interfaces__msg__ScanPoint * lhs, const sick_safetyscanners2_interfaces__msg__ScanPoint * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // angle
  if (lhs->angle != rhs->angle) {
    return false;
  }
  // distance
  if (lhs->distance != rhs->distance) {
    return false;
  }
  // reflectivity
  if (lhs->reflectivity != rhs->reflectivity) {
    return false;
  }
  // valid
  if (lhs->valid != rhs->valid) {
    return false;
  }
  // infinite
  if (lhs->infinite != rhs->infinite) {
    return false;
  }
  // glare
  if (lhs->glare != rhs->glare) {
    return false;
  }
  // reflector
  if (lhs->reflector != rhs->reflector) {
    return false;
  }
  // contamination
  if (lhs->contamination != rhs->contamination) {
    return false;
  }
  // contamination_warning
  if (lhs->contamination_warning != rhs->contamination_warning) {
    return false;
  }
  return true;
}

bool
sick_safetyscanners2_interfaces__msg__ScanPoint__copy(
  const sick_safetyscanners2_interfaces__msg__ScanPoint * input,
  sick_safetyscanners2_interfaces__msg__ScanPoint * output)
{
  if (!input || !output) {
    return false;
  }
  // angle
  output->angle = input->angle;
  // distance
  output->distance = input->distance;
  // reflectivity
  output->reflectivity = input->reflectivity;
  // valid
  output->valid = input->valid;
  // infinite
  output->infinite = input->infinite;
  // glare
  output->glare = input->glare;
  // reflector
  output->reflector = input->reflector;
  // contamination
  output->contamination = input->contamination;
  // contamination_warning
  output->contamination_warning = input->contamination_warning;
  return true;
}

sick_safetyscanners2_interfaces__msg__ScanPoint *
sick_safetyscanners2_interfaces__msg__ScanPoint__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__ScanPoint * msg = (sick_safetyscanners2_interfaces__msg__ScanPoint *)allocator.allocate(sizeof(sick_safetyscanners2_interfaces__msg__ScanPoint), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sick_safetyscanners2_interfaces__msg__ScanPoint));
  bool success = sick_safetyscanners2_interfaces__msg__ScanPoint__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sick_safetyscanners2_interfaces__msg__ScanPoint__destroy(sick_safetyscanners2_interfaces__msg__ScanPoint * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sick_safetyscanners2_interfaces__msg__ScanPoint__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence__init(sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__ScanPoint * data = NULL;

  if (size) {
    data = (sick_safetyscanners2_interfaces__msg__ScanPoint *)allocator.zero_allocate(size, sizeof(sick_safetyscanners2_interfaces__msg__ScanPoint), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sick_safetyscanners2_interfaces__msg__ScanPoint__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sick_safetyscanners2_interfaces__msg__ScanPoint__fini(&data[i - 1]);
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
sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence__fini(sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence * array)
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
      sick_safetyscanners2_interfaces__msg__ScanPoint__fini(&array->data[i]);
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

sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence *
sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence * array = (sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence *)allocator.allocate(sizeof(sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence__destroy(sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence__are_equal(const sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence * lhs, const sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sick_safetyscanners2_interfaces__msg__ScanPoint__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence__copy(
  const sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence * input,
  sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sick_safetyscanners2_interfaces__msg__ScanPoint);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sick_safetyscanners2_interfaces__msg__ScanPoint * data =
      (sick_safetyscanners2_interfaces__msg__ScanPoint *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sick_safetyscanners2_interfaces__msg__ScanPoint__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sick_safetyscanners2_interfaces__msg__ScanPoint__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sick_safetyscanners2_interfaces__msg__ScanPoint__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
