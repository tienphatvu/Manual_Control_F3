// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sick_safetyscanners2_interfaces:msg/MeasurementData.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__MEASUREMENT_DATA__STRUCT_H_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__MEASUREMENT_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'scan_points'
#include "sick_safetyscanners2_interfaces/msg/detail/scan_point__struct.h"

/// Struct defined in msg/MeasurementData in the package sick_safetyscanners2_interfaces.
typedef struct sick_safetyscanners2_interfaces__msg__MeasurementData
{
  uint32_t number_of_beams;
  sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence scan_points;
} sick_safetyscanners2_interfaces__msg__MeasurementData;

// Struct for a sequence of sick_safetyscanners2_interfaces__msg__MeasurementData.
typedef struct sick_safetyscanners2_interfaces__msg__MeasurementData__Sequence
{
  sick_safetyscanners2_interfaces__msg__MeasurementData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sick_safetyscanners2_interfaces__msg__MeasurementData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__MEASUREMENT_DATA__STRUCT_H_
