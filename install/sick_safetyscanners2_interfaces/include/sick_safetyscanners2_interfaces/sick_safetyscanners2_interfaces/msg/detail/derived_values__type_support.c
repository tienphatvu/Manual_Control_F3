// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from sick_safetyscanners2_interfaces:msg/DerivedValues.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "sick_safetyscanners2_interfaces/msg/detail/derived_values__rosidl_typesupport_introspection_c.h"
#include "sick_safetyscanners2_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "sick_safetyscanners2_interfaces/msg/detail/derived_values__functions.h"
#include "sick_safetyscanners2_interfaces/msg/detail/derived_values__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void sick_safetyscanners2_interfaces__msg__DerivedValues__rosidl_typesupport_introspection_c__DerivedValues_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  sick_safetyscanners2_interfaces__msg__DerivedValues__init(message_memory);
}

void sick_safetyscanners2_interfaces__msg__DerivedValues__rosidl_typesupport_introspection_c__DerivedValues_fini_function(void * message_memory)
{
  sick_safetyscanners2_interfaces__msg__DerivedValues__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember sick_safetyscanners2_interfaces__msg__DerivedValues__rosidl_typesupport_introspection_c__DerivedValues_message_member_array[6] = {
  {
    "multiplication_factor",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__DerivedValues, multiplication_factor),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "number_of_beams",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__DerivedValues, number_of_beams),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "scan_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__DerivedValues, scan_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "start_angle",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__DerivedValues, start_angle),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "angular_beam_resolution",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__DerivedValues, angular_beam_resolution),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "interbeam_period",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__DerivedValues, interbeam_period),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers sick_safetyscanners2_interfaces__msg__DerivedValues__rosidl_typesupport_introspection_c__DerivedValues_message_members = {
  "sick_safetyscanners2_interfaces__msg",  // message namespace
  "DerivedValues",  // message name
  6,  // number of fields
  sizeof(sick_safetyscanners2_interfaces__msg__DerivedValues),
  sick_safetyscanners2_interfaces__msg__DerivedValues__rosidl_typesupport_introspection_c__DerivedValues_message_member_array,  // message members
  sick_safetyscanners2_interfaces__msg__DerivedValues__rosidl_typesupport_introspection_c__DerivedValues_init_function,  // function to initialize message memory (memory has to be allocated)
  sick_safetyscanners2_interfaces__msg__DerivedValues__rosidl_typesupport_introspection_c__DerivedValues_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t sick_safetyscanners2_interfaces__msg__DerivedValues__rosidl_typesupport_introspection_c__DerivedValues_message_type_support_handle = {
  0,
  &sick_safetyscanners2_interfaces__msg__DerivedValues__rosidl_typesupport_introspection_c__DerivedValues_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_sick_safetyscanners2_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, msg, DerivedValues)() {
  if (!sick_safetyscanners2_interfaces__msg__DerivedValues__rosidl_typesupport_introspection_c__DerivedValues_message_type_support_handle.typesupport_identifier) {
    sick_safetyscanners2_interfaces__msg__DerivedValues__rosidl_typesupport_introspection_c__DerivedValues_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &sick_safetyscanners2_interfaces__msg__DerivedValues__rosidl_typesupport_introspection_c__DerivedValues_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
