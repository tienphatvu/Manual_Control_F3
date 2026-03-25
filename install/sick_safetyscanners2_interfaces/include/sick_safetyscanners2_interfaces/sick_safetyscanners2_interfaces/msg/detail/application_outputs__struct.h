// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sick_safetyscanners2_interfaces:msg/ApplicationOutputs.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_OUTPUTS__STRUCT_H_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_OUTPUTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'evaluation_path_outputs_eval_out'
// Member 'evaluation_path_outputs_is_safe'
// Member 'evaluation_path_outputs_is_valid'
// Member 'monitoring_case_number_outputs'
// Member 'monitoring_case_number_outputs_flags'
// Member 'resulting_velocity'
// Member 'resulting_velocity_flags'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/ApplicationOutputs in the package sick_safetyscanners2_interfaces.
typedef struct sick_safetyscanners2_interfaces__msg__ApplicationOutputs
{
  rosidl_runtime_c__boolean__Sequence evaluation_path_outputs_eval_out;
  rosidl_runtime_c__boolean__Sequence evaluation_path_outputs_is_safe;
  rosidl_runtime_c__boolean__Sequence evaluation_path_outputs_is_valid;
  rosidl_runtime_c__uint16__Sequence monitoring_case_number_outputs;
  rosidl_runtime_c__boolean__Sequence monitoring_case_number_outputs_flags;
  uint8_t sleep_mode_output;
  bool sleep_mode_output_valid;
  bool error_flag_contamination_warning;
  bool error_flag_contamination_error;
  bool error_flag_manipulation_error;
  bool error_flag_glare;
  bool error_flag_reference_contour_intruded;
  bool error_flag_critical_error;
  bool error_flags_are_valid;
  int16_t linear_velocity_outputs_velocity_0;
  bool linear_velocity_outputs_velocity_0_valid;
  bool linear_velocity_outputs_velocity_0_transmitted_safely;
  int16_t linear_velocity_outputs_velocity_1;
  bool linear_velocity_outputs_velocity_1_valid;
  bool linear_velocity_outputs_velocity_1_transmitted_safely;
  rosidl_runtime_c__int16__Sequence resulting_velocity;
  rosidl_runtime_c__boolean__Sequence resulting_velocity_flags;
} sick_safetyscanners2_interfaces__msg__ApplicationOutputs;

// Struct for a sequence of sick_safetyscanners2_interfaces__msg__ApplicationOutputs.
typedef struct sick_safetyscanners2_interfaces__msg__ApplicationOutputs__Sequence
{
  sick_safetyscanners2_interfaces__msg__ApplicationOutputs * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sick_safetyscanners2_interfaces__msg__ApplicationOutputs__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_OUTPUTS__STRUCT_H_
