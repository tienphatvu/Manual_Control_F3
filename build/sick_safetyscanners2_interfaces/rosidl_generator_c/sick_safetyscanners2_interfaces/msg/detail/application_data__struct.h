// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sick_safetyscanners2_interfaces:msg/ApplicationData.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_DATA__STRUCT_H_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'inputs'
#include "sick_safetyscanners2_interfaces/msg/detail/application_inputs__struct.h"
// Member 'outputs'
#include "sick_safetyscanners2_interfaces/msg/detail/application_outputs__struct.h"

/// Struct defined in msg/ApplicationData in the package sick_safetyscanners2_interfaces.
typedef struct sick_safetyscanners2_interfaces__msg__ApplicationData
{
  sick_safetyscanners2_interfaces__msg__ApplicationInputs inputs;
  sick_safetyscanners2_interfaces__msg__ApplicationOutputs outputs;
} sick_safetyscanners2_interfaces__msg__ApplicationData;

// Struct for a sequence of sick_safetyscanners2_interfaces__msg__ApplicationData.
typedef struct sick_safetyscanners2_interfaces__msg__ApplicationData__Sequence
{
  sick_safetyscanners2_interfaces__msg__ApplicationData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sick_safetyscanners2_interfaces__msg__ApplicationData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_DATA__STRUCT_H_
