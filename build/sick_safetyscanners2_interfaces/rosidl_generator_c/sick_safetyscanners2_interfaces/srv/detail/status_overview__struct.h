// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sick_safetyscanners2_interfaces:srv/StatusOverview.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__SRV__DETAIL__STATUS_OVERVIEW__STRUCT_H_
#define SICK_SAFETYSCANNERS2_INTERFACES__SRV__DETAIL__STATUS_OVERVIEW__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/StatusOverview in the package sick_safetyscanners2_interfaces.
typedef struct sick_safetyscanners2_interfaces__srv__StatusOverview_Request
{
  uint8_t structure_needs_at_least_one_member;
} sick_safetyscanners2_interfaces__srv__StatusOverview_Request;

// Struct for a sequence of sick_safetyscanners2_interfaces__srv__StatusOverview_Request.
typedef struct sick_safetyscanners2_interfaces__srv__StatusOverview_Request__Sequence
{
  sick_safetyscanners2_interfaces__srv__StatusOverview_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sick_safetyscanners2_interfaces__srv__StatusOverview_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'version_c_version'
// Member 'current_time'
// Member 'error_info_time'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/StatusOverview in the package sick_safetyscanners2_interfaces.
typedef struct sick_safetyscanners2_interfaces__srv__StatusOverview_Response
{
  rosidl_runtime_c__String version_c_version;
  uint8_t version_major_version_number;
  uint8_t version_minor_version_number;
  uint8_t version_release_number;
  uint8_t device_state;
  uint8_t config_state;
  uint8_t application_state;
  uint32_t current_time_power_on_count;
  rosidl_runtime_c__String current_time;
  /// for devices without real time clock, also provide the raw time information
  uint32_t current_time_time;
  uint16_t current_time_date;
  uint32_t error_info_code;
  rosidl_runtime_c__String error_info_time;
  /// for devices without real time clock, also provide the raw time information
  uint32_t error_info_time_time;
  uint16_t error_info_time_date;
} sick_safetyscanners2_interfaces__srv__StatusOverview_Response;

// Struct for a sequence of sick_safetyscanners2_interfaces__srv__StatusOverview_Response.
typedef struct sick_safetyscanners2_interfaces__srv__StatusOverview_Response__Sequence
{
  sick_safetyscanners2_interfaces__srv__StatusOverview_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sick_safetyscanners2_interfaces__srv__StatusOverview_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__SRV__DETAIL__STATUS_OVERVIEW__STRUCT_H_
