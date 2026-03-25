// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sick_safetyscanners2_interfaces:msg/DataHeader.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__DATA_HEADER__STRUCT_H_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__DATA_HEADER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/DataHeader in the package sick_safetyscanners2_interfaces.
typedef struct sick_safetyscanners2_interfaces__msg__DataHeader
{
  uint8_t version_version;
  uint8_t version_major_version;
  uint8_t version_minor_version;
  uint8_t version_release;
  uint32_t serial_number_of_device;
  uint32_t serial_number_of_channel_plug;
  uint8_t channel_number;
  uint32_t sequence_number;
  uint32_t scan_number;
  uint16_t timestamp_date;
  uint32_t timestamp_time;
} sick_safetyscanners2_interfaces__msg__DataHeader;

// Struct for a sequence of sick_safetyscanners2_interfaces__msg__DataHeader.
typedef struct sick_safetyscanners2_interfaces__msg__DataHeader__Sequence
{
  sick_safetyscanners2_interfaces__msg__DataHeader * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sick_safetyscanners2_interfaces__msg__DataHeader__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__DATA_HEADER__STRUCT_H_
