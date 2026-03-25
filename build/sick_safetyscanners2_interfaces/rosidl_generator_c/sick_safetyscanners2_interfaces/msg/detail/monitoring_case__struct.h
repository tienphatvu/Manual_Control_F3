// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sick_safetyscanners2_interfaces:msg/MonitoringCase.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__MONITORING_CASE__STRUCT_H_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__MONITORING_CASE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'fields'
// Member 'fields_valid'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/MonitoringCase in the package sick_safetyscanners2_interfaces.
typedef struct sick_safetyscanners2_interfaces__msg__MonitoringCase
{
  int32_t monitoring_case_number;
  rosidl_runtime_c__int32__Sequence fields;
  rosidl_runtime_c__boolean__Sequence fields_valid;
} sick_safetyscanners2_interfaces__msg__MonitoringCase;

// Struct for a sequence of sick_safetyscanners2_interfaces__msg__MonitoringCase.
typedef struct sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence
{
  sick_safetyscanners2_interfaces__msg__MonitoringCase * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__MONITORING_CASE__STRUCT_H_
