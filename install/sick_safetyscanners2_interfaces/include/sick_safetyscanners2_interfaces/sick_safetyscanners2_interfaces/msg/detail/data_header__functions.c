// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from sick_safetyscanners2_interfaces:msg/DataHeader.idl
// generated code does not contain a copyright notice
#include "sick_safetyscanners2_interfaces/msg/detail/data_header__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
sick_safetyscanners2_interfaces__msg__DataHeader__init(sick_safetyscanners2_interfaces__msg__DataHeader * msg)
{
  if (!msg) {
    return false;
  }
  // version_version
  // version_major_version
  // version_minor_version
  // version_release
  // serial_number_of_device
  // serial_number_of_channel_plug
  // channel_number
  // sequence_number
  // scan_number
  // timestamp_date
  // timestamp_time
  return true;
}

void
sick_safetyscanners2_interfaces__msg__DataHeader__fini(sick_safetyscanners2_interfaces__msg__DataHeader * msg)
{
  if (!msg) {
    return;
  }
  // version_version
  // version_major_version
  // version_minor_version
  // version_release
  // serial_number_of_device
  // serial_number_of_channel_plug
  // channel_number
  // sequence_number
  // scan_number
  // timestamp_date
  // timestamp_time
}

bool
sick_safetyscanners2_interfaces__msg__DataHeader__are_equal(const sick_safetyscanners2_interfaces__msg__DataHeader * lhs, const sick_safetyscanners2_interfaces__msg__DataHeader * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // version_version
  if (lhs->version_version != rhs->version_version) {
    return false;
  }
  // version_major_version
  if (lhs->version_major_version != rhs->version_major_version) {
    return false;
  }
  // version_minor_version
  if (lhs->version_minor_version != rhs->version_minor_version) {
    return false;
  }
  // version_release
  if (lhs->version_release != rhs->version_release) {
    return false;
  }
  // serial_number_of_device
  if (lhs->serial_number_of_device != rhs->serial_number_of_device) {
    return false;
  }
  // serial_number_of_channel_plug
  if (lhs->serial_number_of_channel_plug != rhs->serial_number_of_channel_plug) {
    return false;
  }
  // channel_number
  if (lhs->channel_number != rhs->channel_number) {
    return false;
  }
  // sequence_number
  if (lhs->sequence_number != rhs->sequence_number) {
    return false;
  }
  // scan_number
  if (lhs->scan_number != rhs->scan_number) {
    return false;
  }
  // timestamp_date
  if (lhs->timestamp_date != rhs->timestamp_date) {
    return false;
  }
  // timestamp_time
  if (lhs->timestamp_time != rhs->timestamp_time) {
    return false;
  }
  return true;
}

bool
sick_safetyscanners2_interfaces__msg__DataHeader__copy(
  const sick_safetyscanners2_interfaces__msg__DataHeader * input,
  sick_safetyscanners2_interfaces__msg__DataHeader * output)
{
  if (!input || !output) {
    return false;
  }
  // version_version
  output->version_version = input->version_version;
  // version_major_version
  output->version_major_version = input->version_major_version;
  // version_minor_version
  output->version_minor_version = input->version_minor_version;
  // version_release
  output->version_release = input->version_release;
  // serial_number_of_device
  output->serial_number_of_device = input->serial_number_of_device;
  // serial_number_of_channel_plug
  output->serial_number_of_channel_plug = input->serial_number_of_channel_plug;
  // channel_number
  output->channel_number = input->channel_number;
  // sequence_number
  output->sequence_number = input->sequence_number;
  // scan_number
  output->scan_number = input->scan_number;
  // timestamp_date
  output->timestamp_date = input->timestamp_date;
  // timestamp_time
  output->timestamp_time = input->timestamp_time;
  return true;
}

sick_safetyscanners2_interfaces__msg__DataHeader *
sick_safetyscanners2_interfaces__msg__DataHeader__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__DataHeader * msg = (sick_safetyscanners2_interfaces__msg__DataHeader *)allocator.allocate(sizeof(sick_safetyscanners2_interfaces__msg__DataHeader), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sick_safetyscanners2_interfaces__msg__DataHeader));
  bool success = sick_safetyscanners2_interfaces__msg__DataHeader__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sick_safetyscanners2_interfaces__msg__DataHeader__destroy(sick_safetyscanners2_interfaces__msg__DataHeader * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sick_safetyscanners2_interfaces__msg__DataHeader__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sick_safetyscanners2_interfaces__msg__DataHeader__Sequence__init(sick_safetyscanners2_interfaces__msg__DataHeader__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__DataHeader * data = NULL;

  if (size) {
    data = (sick_safetyscanners2_interfaces__msg__DataHeader *)allocator.zero_allocate(size, sizeof(sick_safetyscanners2_interfaces__msg__DataHeader), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sick_safetyscanners2_interfaces__msg__DataHeader__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sick_safetyscanners2_interfaces__msg__DataHeader__fini(&data[i - 1]);
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
sick_safetyscanners2_interfaces__msg__DataHeader__Sequence__fini(sick_safetyscanners2_interfaces__msg__DataHeader__Sequence * array)
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
      sick_safetyscanners2_interfaces__msg__DataHeader__fini(&array->data[i]);
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

sick_safetyscanners2_interfaces__msg__DataHeader__Sequence *
sick_safetyscanners2_interfaces__msg__DataHeader__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sick_safetyscanners2_interfaces__msg__DataHeader__Sequence * array = (sick_safetyscanners2_interfaces__msg__DataHeader__Sequence *)allocator.allocate(sizeof(sick_safetyscanners2_interfaces__msg__DataHeader__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sick_safetyscanners2_interfaces__msg__DataHeader__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sick_safetyscanners2_interfaces__msg__DataHeader__Sequence__destroy(sick_safetyscanners2_interfaces__msg__DataHeader__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sick_safetyscanners2_interfaces__msg__DataHeader__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sick_safetyscanners2_interfaces__msg__DataHeader__Sequence__are_equal(const sick_safetyscanners2_interfaces__msg__DataHeader__Sequence * lhs, const sick_safetyscanners2_interfaces__msg__DataHeader__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sick_safetyscanners2_interfaces__msg__DataHeader__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sick_safetyscanners2_interfaces__msg__DataHeader__Sequence__copy(
  const sick_safetyscanners2_interfaces__msg__DataHeader__Sequence * input,
  sick_safetyscanners2_interfaces__msg__DataHeader__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sick_safetyscanners2_interfaces__msg__DataHeader);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sick_safetyscanners2_interfaces__msg__DataHeader * data =
      (sick_safetyscanners2_interfaces__msg__DataHeader *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sick_safetyscanners2_interfaces__msg__DataHeader__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sick_safetyscanners2_interfaces__msg__DataHeader__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sick_safetyscanners2_interfaces__msg__DataHeader__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
