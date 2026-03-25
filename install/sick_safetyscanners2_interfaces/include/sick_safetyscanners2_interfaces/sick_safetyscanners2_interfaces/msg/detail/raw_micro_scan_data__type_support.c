// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from sick_safetyscanners2_interfaces:msg/RawMicroScanData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "sick_safetyscanners2_interfaces/msg/detail/raw_micro_scan_data__rosidl_typesupport_introspection_c.h"
#include "sick_safetyscanners2_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "sick_safetyscanners2_interfaces/msg/detail/raw_micro_scan_data__functions.h"
#include "sick_safetyscanners2_interfaces/msg/detail/raw_micro_scan_data__struct.h"


// Include directives for member types
// Member `header`
#include "sick_safetyscanners2_interfaces/msg/data_header.h"
// Member `header`
#include "sick_safetyscanners2_interfaces/msg/detail/data_header__rosidl_typesupport_introspection_c.h"
// Member `derived_values`
#include "sick_safetyscanners2_interfaces/msg/derived_values.h"
// Member `derived_values`
#include "sick_safetyscanners2_interfaces/msg/detail/derived_values__rosidl_typesupport_introspection_c.h"
// Member `general_system_state`
#include "sick_safetyscanners2_interfaces/msg/general_system_state.h"
// Member `general_system_state`
#include "sick_safetyscanners2_interfaces/msg/detail/general_system_state__rosidl_typesupport_introspection_c.h"
// Member `measurement_data`
#include "sick_safetyscanners2_interfaces/msg/measurement_data.h"
// Member `measurement_data`
#include "sick_safetyscanners2_interfaces/msg/detail/measurement_data__rosidl_typesupport_introspection_c.h"
// Member `intrusion_data`
#include "sick_safetyscanners2_interfaces/msg/intrusion_data.h"
// Member `intrusion_data`
#include "sick_safetyscanners2_interfaces/msg/detail/intrusion_data__rosidl_typesupport_introspection_c.h"
// Member `application_data`
#include "sick_safetyscanners2_interfaces/msg/application_data.h"
// Member `application_data`
#include "sick_safetyscanners2_interfaces/msg/detail/application_data__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void sick_safetyscanners2_interfaces__msg__RawMicroScanData__rosidl_typesupport_introspection_c__RawMicroScanData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  sick_safetyscanners2_interfaces__msg__RawMicroScanData__init(message_memory);
}

void sick_safetyscanners2_interfaces__msg__RawMicroScanData__rosidl_typesupport_introspection_c__RawMicroScanData_fini_function(void * message_memory)
{
  sick_safetyscanners2_interfaces__msg__RawMicroScanData__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember sick_safetyscanners2_interfaces__msg__RawMicroScanData__rosidl_typesupport_introspection_c__RawMicroScanData_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__RawMicroScanData, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "derived_values",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__RawMicroScanData, derived_values),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "general_system_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__RawMicroScanData, general_system_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "measurement_data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__RawMicroScanData, measurement_data),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "intrusion_data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__RawMicroScanData, intrusion_data),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "application_data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__RawMicroScanData, application_data),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers sick_safetyscanners2_interfaces__msg__RawMicroScanData__rosidl_typesupport_introspection_c__RawMicroScanData_message_members = {
  "sick_safetyscanners2_interfaces__msg",  // message namespace
  "RawMicroScanData",  // message name
  6,  // number of fields
  sizeof(sick_safetyscanners2_interfaces__msg__RawMicroScanData),
  sick_safetyscanners2_interfaces__msg__RawMicroScanData__rosidl_typesupport_introspection_c__RawMicroScanData_message_member_array,  // message members
  sick_safetyscanners2_interfaces__msg__RawMicroScanData__rosidl_typesupport_introspection_c__RawMicroScanData_init_function,  // function to initialize message memory (memory has to be allocated)
  sick_safetyscanners2_interfaces__msg__RawMicroScanData__rosidl_typesupport_introspection_c__RawMicroScanData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t sick_safetyscanners2_interfaces__msg__RawMicroScanData__rosidl_typesupport_introspection_c__RawMicroScanData_message_type_support_handle = {
  0,
  &sick_safetyscanners2_interfaces__msg__RawMicroScanData__rosidl_typesupport_introspection_c__RawMicroScanData_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_sick_safetyscanners2_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, msg, RawMicroScanData)() {
  sick_safetyscanners2_interfaces__msg__RawMicroScanData__rosidl_typesupport_introspection_c__RawMicroScanData_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, msg, DataHeader)();
  sick_safetyscanners2_interfaces__msg__RawMicroScanData__rosidl_typesupport_introspection_c__RawMicroScanData_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, msg, DerivedValues)();
  sick_safetyscanners2_interfaces__msg__RawMicroScanData__rosidl_typesupport_introspection_c__RawMicroScanData_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, msg, GeneralSystemState)();
  sick_safetyscanners2_interfaces__msg__RawMicroScanData__rosidl_typesupport_introspection_c__RawMicroScanData_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, msg, MeasurementData)();
  sick_safetyscanners2_interfaces__msg__RawMicroScanData__rosidl_typesupport_introspection_c__RawMicroScanData_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, msg, IntrusionData)();
  sick_safetyscanners2_interfaces__msg__RawMicroScanData__rosidl_typesupport_introspection_c__RawMicroScanData_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, msg, ApplicationData)();
  if (!sick_safetyscanners2_interfaces__msg__RawMicroScanData__rosidl_typesupport_introspection_c__RawMicroScanData_message_type_support_handle.typesupport_identifier) {
    sick_safetyscanners2_interfaces__msg__RawMicroScanData__rosidl_typesupport_introspection_c__RawMicroScanData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &sick_safetyscanners2_interfaces__msg__RawMicroScanData__rosidl_typesupport_introspection_c__RawMicroScanData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
