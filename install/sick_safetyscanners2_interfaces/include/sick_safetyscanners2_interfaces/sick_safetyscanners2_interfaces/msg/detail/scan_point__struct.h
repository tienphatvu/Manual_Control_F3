// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sick_safetyscanners2_interfaces:msg/ScanPoint.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__SCAN_POINT__STRUCT_H_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__SCAN_POINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ScanPoint in the package sick_safetyscanners2_interfaces.
typedef struct sick_safetyscanners2_interfaces__msg__ScanPoint
{
  float angle;
  uint16_t distance;
  uint8_t reflectivity;
  bool valid;
  bool infinite;
  bool glare;
  bool reflector;
  bool contamination;
  bool contamination_warning;
} sick_safetyscanners2_interfaces__msg__ScanPoint;

// Struct for a sequence of sick_safetyscanners2_interfaces__msg__ScanPoint.
typedef struct sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence
{
  sick_safetyscanners2_interfaces__msg__ScanPoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__SCAN_POINT__STRUCT_H_
