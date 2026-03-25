// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sick_safetyscanners2_interfaces:msg/IntrusionDatum.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__INTRUSION_DATUM__STRUCT_H_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__INTRUSION_DATUM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'flags'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/IntrusionDatum in the package sick_safetyscanners2_interfaces.
typedef struct sick_safetyscanners2_interfaces__msg__IntrusionDatum
{
  uint32_t size;
  rosidl_runtime_c__boolean__Sequence flags;
} sick_safetyscanners2_interfaces__msg__IntrusionDatum;

// Struct for a sequence of sick_safetyscanners2_interfaces__msg__IntrusionDatum.
typedef struct sick_safetyscanners2_interfaces__msg__IntrusionDatum__Sequence
{
  sick_safetyscanners2_interfaces__msg__IntrusionDatum * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sick_safetyscanners2_interfaces__msg__IntrusionDatum__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__INTRUSION_DATUM__STRUCT_H_
