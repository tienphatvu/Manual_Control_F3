// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sick_safetyscanners2_interfaces:msg/Field.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__FIELD__STRUCT_H_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__FIELD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'ranges'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Field in the package sick_safetyscanners2_interfaces.
typedef struct sick_safetyscanners2_interfaces__msg__Field
{
  rosidl_runtime_c__float__Sequence ranges;
  float start_angle;
  float angular_resolution;
  bool protective_field;
} sick_safetyscanners2_interfaces__msg__Field;

// Struct for a sequence of sick_safetyscanners2_interfaces__msg__Field.
typedef struct sick_safetyscanners2_interfaces__msg__Field__Sequence
{
  sick_safetyscanners2_interfaces__msg__Field * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sick_safetyscanners2_interfaces__msg__Field__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__FIELD__STRUCT_H_
