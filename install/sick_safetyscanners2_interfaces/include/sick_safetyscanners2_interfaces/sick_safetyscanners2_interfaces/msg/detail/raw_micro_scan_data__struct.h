// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sick_safetyscanners2_interfaces:msg/RawMicroScanData.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__RAW_MICRO_SCAN_DATA__STRUCT_H_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__RAW_MICRO_SCAN_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "sick_safetyscanners2_interfaces/msg/detail/data_header__struct.h"
// Member 'derived_values'
#include "sick_safetyscanners2_interfaces/msg/detail/derived_values__struct.h"
// Member 'general_system_state'
#include "sick_safetyscanners2_interfaces/msg/detail/general_system_state__struct.h"
// Member 'measurement_data'
#include "sick_safetyscanners2_interfaces/msg/detail/measurement_data__struct.h"
// Member 'intrusion_data'
#include "sick_safetyscanners2_interfaces/msg/detail/intrusion_data__struct.h"
// Member 'application_data'
#include "sick_safetyscanners2_interfaces/msg/detail/application_data__struct.h"

/// Struct defined in msg/RawMicroScanData in the package sick_safetyscanners2_interfaces.
typedef struct sick_safetyscanners2_interfaces__msg__RawMicroScanData
{
  sick_safetyscanners2_interfaces__msg__DataHeader header;
  sick_safetyscanners2_interfaces__msg__DerivedValues derived_values;
  sick_safetyscanners2_interfaces__msg__GeneralSystemState general_system_state;
  sick_safetyscanners2_interfaces__msg__MeasurementData measurement_data;
  sick_safetyscanners2_interfaces__msg__IntrusionData intrusion_data;
  sick_safetyscanners2_interfaces__msg__ApplicationData application_data;
} sick_safetyscanners2_interfaces__msg__RawMicroScanData;

// Struct for a sequence of sick_safetyscanners2_interfaces__msg__RawMicroScanData.
typedef struct sick_safetyscanners2_interfaces__msg__RawMicroScanData__Sequence
{
  sick_safetyscanners2_interfaces__msg__RawMicroScanData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sick_safetyscanners2_interfaces__msg__RawMicroScanData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__RAW_MICRO_SCAN_DATA__STRUCT_H_
