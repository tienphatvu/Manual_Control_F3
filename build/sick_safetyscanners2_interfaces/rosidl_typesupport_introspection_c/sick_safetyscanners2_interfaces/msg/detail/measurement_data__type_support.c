// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from sick_safetyscanners2_interfaces:msg/MeasurementData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "sick_safetyscanners2_interfaces/msg/detail/measurement_data__rosidl_typesupport_introspection_c.h"
#include "sick_safetyscanners2_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "sick_safetyscanners2_interfaces/msg/detail/measurement_data__functions.h"
#include "sick_safetyscanners2_interfaces/msg/detail/measurement_data__struct.h"


// Include directives for member types
// Member `scan_points`
#include "sick_safetyscanners2_interfaces/msg/scan_point.h"
// Member `scan_points`
#include "sick_safetyscanners2_interfaces/msg/detail/scan_point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void sick_safetyscanners2_interfaces__msg__MeasurementData__rosidl_typesupport_introspection_c__MeasurementData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  sick_safetyscanners2_interfaces__msg__MeasurementData__init(message_memory);
}

void sick_safetyscanners2_interfaces__msg__MeasurementData__rosidl_typesupport_introspection_c__MeasurementData_fini_function(void * message_memory)
{
  sick_safetyscanners2_interfaces__msg__MeasurementData__fini(message_memory);
}

size_t sick_safetyscanners2_interfaces__msg__MeasurementData__rosidl_typesupport_introspection_c__size_function__MeasurementData__scan_points(
  const void * untyped_member)
{
  const sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence * member =
    (const sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence *)(untyped_member);
  return member->size;
}

const void * sick_safetyscanners2_interfaces__msg__MeasurementData__rosidl_typesupport_introspection_c__get_const_function__MeasurementData__scan_points(
  const void * untyped_member, size_t index)
{
  const sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence * member =
    (const sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence *)(untyped_member);
  return &member->data[index];
}

void * sick_safetyscanners2_interfaces__msg__MeasurementData__rosidl_typesupport_introspection_c__get_function__MeasurementData__scan_points(
  void * untyped_member, size_t index)
{
  sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence * member =
    (sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence *)(untyped_member);
  return &member->data[index];
}

void sick_safetyscanners2_interfaces__msg__MeasurementData__rosidl_typesupport_introspection_c__fetch_function__MeasurementData__scan_points(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const sick_safetyscanners2_interfaces__msg__ScanPoint * item =
    ((const sick_safetyscanners2_interfaces__msg__ScanPoint *)
    sick_safetyscanners2_interfaces__msg__MeasurementData__rosidl_typesupport_introspection_c__get_const_function__MeasurementData__scan_points(untyped_member, index));
  sick_safetyscanners2_interfaces__msg__ScanPoint * value =
    (sick_safetyscanners2_interfaces__msg__ScanPoint *)(untyped_value);
  *value = *item;
}

void sick_safetyscanners2_interfaces__msg__MeasurementData__rosidl_typesupport_introspection_c__assign_function__MeasurementData__scan_points(
  void * untyped_member, size_t index, const void * untyped_value)
{
  sick_safetyscanners2_interfaces__msg__ScanPoint * item =
    ((sick_safetyscanners2_interfaces__msg__ScanPoint *)
    sick_safetyscanners2_interfaces__msg__MeasurementData__rosidl_typesupport_introspection_c__get_function__MeasurementData__scan_points(untyped_member, index));
  const sick_safetyscanners2_interfaces__msg__ScanPoint * value =
    (const sick_safetyscanners2_interfaces__msg__ScanPoint *)(untyped_value);
  *item = *value;
}

bool sick_safetyscanners2_interfaces__msg__MeasurementData__rosidl_typesupport_introspection_c__resize_function__MeasurementData__scan_points(
  void * untyped_member, size_t size)
{
  sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence * member =
    (sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence *)(untyped_member);
  sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence__fini(member);
  return sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember sick_safetyscanners2_interfaces__msg__MeasurementData__rosidl_typesupport_introspection_c__MeasurementData_message_member_array[2] = {
  {
    "number_of_beams",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__MeasurementData, number_of_beams),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "scan_points",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__MeasurementData, scan_points),  // bytes offset in struct
    NULL,  // default value
    sick_safetyscanners2_interfaces__msg__MeasurementData__rosidl_typesupport_introspection_c__size_function__MeasurementData__scan_points,  // size() function pointer
    sick_safetyscanners2_interfaces__msg__MeasurementData__rosidl_typesupport_introspection_c__get_const_function__MeasurementData__scan_points,  // get_const(index) function pointer
    sick_safetyscanners2_interfaces__msg__MeasurementData__rosidl_typesupport_introspection_c__get_function__MeasurementData__scan_points,  // get(index) function pointer
    sick_safetyscanners2_interfaces__msg__MeasurementData__rosidl_typesupport_introspection_c__fetch_function__MeasurementData__scan_points,  // fetch(index, &value) function pointer
    sick_safetyscanners2_interfaces__msg__MeasurementData__rosidl_typesupport_introspection_c__assign_function__MeasurementData__scan_points,  // assign(index, value) function pointer
    sick_safetyscanners2_interfaces__msg__MeasurementData__rosidl_typesupport_introspection_c__resize_function__MeasurementData__scan_points  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers sick_safetyscanners2_interfaces__msg__MeasurementData__rosidl_typesupport_introspection_c__MeasurementData_message_members = {
  "sick_safetyscanners2_interfaces__msg",  // message namespace
  "MeasurementData",  // message name
  2,  // number of fields
  sizeof(sick_safetyscanners2_interfaces__msg__MeasurementData),
  sick_safetyscanners2_interfaces__msg__MeasurementData__rosidl_typesupport_introspection_c__MeasurementData_message_member_array,  // message members
  sick_safetyscanners2_interfaces__msg__MeasurementData__rosidl_typesupport_introspection_c__MeasurementData_init_function,  // function to initialize message memory (memory has to be allocated)
  sick_safetyscanners2_interfaces__msg__MeasurementData__rosidl_typesupport_introspection_c__MeasurementData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t sick_safetyscanners2_interfaces__msg__MeasurementData__rosidl_typesupport_introspection_c__MeasurementData_message_type_support_handle = {
  0,
  &sick_safetyscanners2_interfaces__msg__MeasurementData__rosidl_typesupport_introspection_c__MeasurementData_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_sick_safetyscanners2_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, msg, MeasurementData)() {
  sick_safetyscanners2_interfaces__msg__MeasurementData__rosidl_typesupport_introspection_c__MeasurementData_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, msg, ScanPoint)();
  if (!sick_safetyscanners2_interfaces__msg__MeasurementData__rosidl_typesupport_introspection_c__MeasurementData_message_type_support_handle.typesupport_identifier) {
    sick_safetyscanners2_interfaces__msg__MeasurementData__rosidl_typesupport_introspection_c__MeasurementData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &sick_safetyscanners2_interfaces__msg__MeasurementData__rosidl_typesupport_introspection_c__MeasurementData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
