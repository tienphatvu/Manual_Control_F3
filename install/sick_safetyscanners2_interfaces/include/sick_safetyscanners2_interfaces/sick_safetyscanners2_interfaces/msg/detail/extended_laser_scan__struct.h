// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sick_safetyscanners2_interfaces:msg/ExtendedLaserScan.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__EXTENDED_LASER_SCAN__STRUCT_H_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__EXTENDED_LASER_SCAN__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'laser_scan'
#include "sensor_msgs/msg/detail/laser_scan__struct.h"
// Member 'reflektor_status'
// Member 'reflektor_median'
// Member 'intrusion'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/ExtendedLaserScan in the package sick_safetyscanners2_interfaces.
typedef struct sick_safetyscanners2_interfaces__msg__ExtendedLaserScan
{
  sensor_msgs__msg__LaserScan laser_scan;
  rosidl_runtime_c__boolean__Sequence reflektor_status;
  rosidl_runtime_c__boolean__Sequence reflektor_median;
  rosidl_runtime_c__boolean__Sequence intrusion;
} sick_safetyscanners2_interfaces__msg__ExtendedLaserScan;

// Struct for a sequence of sick_safetyscanners2_interfaces__msg__ExtendedLaserScan.
typedef struct sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__Sequence
{
  sick_safetyscanners2_interfaces__msg__ExtendedLaserScan * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__EXTENDED_LASER_SCAN__STRUCT_H_
