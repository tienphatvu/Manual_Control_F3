// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sick_safetyscanners2_interfaces:srv/FieldData.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__SRV__DETAIL__FIELD_DATA__STRUCT_H_
#define SICK_SAFETYSCANNERS2_INTERFACES__SRV__DETAIL__FIELD_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/FieldData in the package sick_safetyscanners2_interfaces.
typedef struct sick_safetyscanners2_interfaces__srv__FieldData_Request
{
  uint8_t structure_needs_at_least_one_member;
} sick_safetyscanners2_interfaces__srv__FieldData_Request;

// Struct for a sequence of sick_safetyscanners2_interfaces__srv__FieldData_Request.
typedef struct sick_safetyscanners2_interfaces__srv__FieldData_Request__Sequence
{
  sick_safetyscanners2_interfaces__srv__FieldData_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sick_safetyscanners2_interfaces__srv__FieldData_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'fields'
#include "sick_safetyscanners2_interfaces/msg/detail/field__struct.h"
// Member 'device_name'
#include "rosidl_runtime_c/string.h"
// Member 'monitoring_cases'
#include "sick_safetyscanners2_interfaces/msg/detail/monitoring_case__struct.h"

/// Struct defined in srv/FieldData in the package sick_safetyscanners2_interfaces.
typedef struct sick_safetyscanners2_interfaces__srv__FieldData_Response
{
  sick_safetyscanners2_interfaces__msg__Field__Sequence fields;
  rosidl_runtime_c__String device_name;
  sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence monitoring_cases;
} sick_safetyscanners2_interfaces__srv__FieldData_Response;

// Struct for a sequence of sick_safetyscanners2_interfaces__srv__FieldData_Response.
typedef struct sick_safetyscanners2_interfaces__srv__FieldData_Response__Sequence
{
  sick_safetyscanners2_interfaces__srv__FieldData_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sick_safetyscanners2_interfaces__srv__FieldData_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__SRV__DETAIL__FIELD_DATA__STRUCT_H_
