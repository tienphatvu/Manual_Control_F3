// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sick_safetyscanners2_interfaces:msg/GeneralSystemState.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__GENERAL_SYSTEM_STATE__STRUCT_H_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__GENERAL_SYSTEM_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'safe_cut_off_path'
// Member 'non_safe_cut_off_path'
// Member 'reset_required_cut_off_path'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/GeneralSystemState in the package sick_safetyscanners2_interfaces.
typedef struct sick_safetyscanners2_interfaces__msg__GeneralSystemState
{
  bool run_mode_active;
  bool standby_mode_active;
  bool contamination_warning;
  bool contamination_error;
  bool reference_contour_status;
  bool manipulation_status;
  rosidl_runtime_c__boolean__Sequence safe_cut_off_path;
  rosidl_runtime_c__boolean__Sequence non_safe_cut_off_path;
  rosidl_runtime_c__boolean__Sequence reset_required_cut_off_path;
  uint8_t current_monitoring_case_no_table_1;
  uint8_t current_monitoring_case_no_table_2;
  uint8_t current_monitoring_case_no_table_3;
  uint8_t current_monitoring_case_no_table_4;
  bool application_error;
  bool device_error;
} sick_safetyscanners2_interfaces__msg__GeneralSystemState;

// Struct for a sequence of sick_safetyscanners2_interfaces__msg__GeneralSystemState.
typedef struct sick_safetyscanners2_interfaces__msg__GeneralSystemState__Sequence
{
  sick_safetyscanners2_interfaces__msg__GeneralSystemState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sick_safetyscanners2_interfaces__msg__GeneralSystemState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__GENERAL_SYSTEM_STATE__STRUCT_H_
