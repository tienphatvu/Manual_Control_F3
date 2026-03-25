// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sick_safetyscanners2_interfaces:msg/IntrusionData.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__INTRUSION_DATA__STRUCT_H_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__INTRUSION_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "sick_safetyscanners2_interfaces/msg/detail/intrusion_datum__struct.h"

/// Struct defined in msg/IntrusionData in the package sick_safetyscanners2_interfaces.
typedef struct sick_safetyscanners2_interfaces__msg__IntrusionData
{
  sick_safetyscanners2_interfaces__msg__IntrusionDatum__Sequence data;
} sick_safetyscanners2_interfaces__msg__IntrusionData;

// Struct for a sequence of sick_safetyscanners2_interfaces__msg__IntrusionData.
typedef struct sick_safetyscanners2_interfaces__msg__IntrusionData__Sequence
{
  sick_safetyscanners2_interfaces__msg__IntrusionData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sick_safetyscanners2_interfaces__msg__IntrusionData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__INTRUSION_DATA__STRUCT_H_
