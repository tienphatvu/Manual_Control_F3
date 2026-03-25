// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sick_safetyscanners2_interfaces:msg/DerivedValues.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__DERIVED_VALUES__STRUCT_H_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__DERIVED_VALUES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/DerivedValues in the package sick_safetyscanners2_interfaces.
typedef struct sick_safetyscanners2_interfaces__msg__DerivedValues
{
  uint16_t multiplication_factor;
  uint16_t number_of_beams;
  uint16_t scan_time;
  float start_angle;
  float angular_beam_resolution;
  uint32_t interbeam_period;
} sick_safetyscanners2_interfaces__msg__DerivedValues;

// Struct for a sequence of sick_safetyscanners2_interfaces__msg__DerivedValues.
typedef struct sick_safetyscanners2_interfaces__msg__DerivedValues__Sequence
{
  sick_safetyscanners2_interfaces__msg__DerivedValues * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sick_safetyscanners2_interfaces__msg__DerivedValues__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__DERIVED_VALUES__STRUCT_H_
