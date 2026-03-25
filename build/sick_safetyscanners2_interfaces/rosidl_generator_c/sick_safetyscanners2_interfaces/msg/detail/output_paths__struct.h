// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sick_safetyscanners2_interfaces:msg/OutputPaths.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__OUTPUT_PATHS__STRUCT_H_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__OUTPUT_PATHS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'status'
// Member 'is_safe'
// Member 'is_valid'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/OutputPaths in the package sick_safetyscanners2_interfaces.
typedef struct sick_safetyscanners2_interfaces__msg__OutputPaths
{
  rosidl_runtime_c__boolean__Sequence status;
  rosidl_runtime_c__boolean__Sequence is_safe;
  rosidl_runtime_c__boolean__Sequence is_valid;
  int32_t active_monitoring_case;
} sick_safetyscanners2_interfaces__msg__OutputPaths;

// Struct for a sequence of sick_safetyscanners2_interfaces__msg__OutputPaths.
typedef struct sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence
{
  sick_safetyscanners2_interfaces__msg__OutputPaths * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__OUTPUT_PATHS__STRUCT_H_
