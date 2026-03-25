// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sick_safetyscanners2_interfaces:msg/ApplicationInputs.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_INPUTS__STRUCT_H_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_INPUTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'unsafe_inputs_input_sources'
// Member 'unsafe_inputs_flags'
// Member 'monitoring_case_number_inputs'
// Member 'monitoring_case_number_inputs_flags'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/ApplicationInputs in the package sick_safetyscanners2_interfaces.
typedef struct sick_safetyscanners2_interfaces__msg__ApplicationInputs
{
  rosidl_runtime_c__boolean__Sequence unsafe_inputs_input_sources;
  rosidl_runtime_c__boolean__Sequence unsafe_inputs_flags;
  rosidl_runtime_c__uint16__Sequence monitoring_case_number_inputs;
  rosidl_runtime_c__boolean__Sequence monitoring_case_number_inputs_flags;
  int16_t linear_velocity_inputs_velocity_0;
  bool linear_velocity_inputs_velocity_0_valid;
  bool linear_velocity_inputs_velocity_0_transmitted_safely;
  int16_t linear_velocity_inputs_velocity_1;
  bool linear_velocity_inputs_velocity_1_valid;
  bool linear_velocity_inputs_velocity_1_transmitted_safely;
  uint8_t sleep_mode_input;
} sick_safetyscanners2_interfaces__msg__ApplicationInputs;

// Struct for a sequence of sick_safetyscanners2_interfaces__msg__ApplicationInputs.
typedef struct sick_safetyscanners2_interfaces__msg__ApplicationInputs__Sequence
{
  sick_safetyscanners2_interfaces__msg__ApplicationInputs * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sick_safetyscanners2_interfaces__msg__ApplicationInputs__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_INPUTS__STRUCT_H_
