// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from sick_safetyscanners2_interfaces:msg/IntrusionDatum.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "sick_safetyscanners2_interfaces/msg/detail/intrusion_datum__rosidl_typesupport_introspection_c.h"
#include "sick_safetyscanners2_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "sick_safetyscanners2_interfaces/msg/detail/intrusion_datum__functions.h"
#include "sick_safetyscanners2_interfaces/msg/detail/intrusion_datum__struct.h"


// Include directives for member types
// Member `flags`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void sick_safetyscanners2_interfaces__msg__IntrusionDatum__rosidl_typesupport_introspection_c__IntrusionDatum_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  sick_safetyscanners2_interfaces__msg__IntrusionDatum__init(message_memory);
}

void sick_safetyscanners2_interfaces__msg__IntrusionDatum__rosidl_typesupport_introspection_c__IntrusionDatum_fini_function(void * message_memory)
{
  sick_safetyscanners2_interfaces__msg__IntrusionDatum__fini(message_memory);
}

size_t sick_safetyscanners2_interfaces__msg__IntrusionDatum__rosidl_typesupport_introspection_c__size_function__IntrusionDatum__flags(
  const void * untyped_member)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return member->size;
}

const void * sick_safetyscanners2_interfaces__msg__IntrusionDatum__rosidl_typesupport_introspection_c__get_const_function__IntrusionDatum__flags(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void * sick_safetyscanners2_interfaces__msg__IntrusionDatum__rosidl_typesupport_introspection_c__get_function__IntrusionDatum__flags(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void sick_safetyscanners2_interfaces__msg__IntrusionDatum__rosidl_typesupport_introspection_c__fetch_function__IntrusionDatum__flags(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    sick_safetyscanners2_interfaces__msg__IntrusionDatum__rosidl_typesupport_introspection_c__get_const_function__IntrusionDatum__flags(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void sick_safetyscanners2_interfaces__msg__IntrusionDatum__rosidl_typesupport_introspection_c__assign_function__IntrusionDatum__flags(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    sick_safetyscanners2_interfaces__msg__IntrusionDatum__rosidl_typesupport_introspection_c__get_function__IntrusionDatum__flags(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

bool sick_safetyscanners2_interfaces__msg__IntrusionDatum__rosidl_typesupport_introspection_c__resize_function__IntrusionDatum__flags(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  rosidl_runtime_c__boolean__Sequence__fini(member);
  return rosidl_runtime_c__boolean__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember sick_safetyscanners2_interfaces__msg__IntrusionDatum__rosidl_typesupport_introspection_c__IntrusionDatum_message_member_array[2] = {
  {
    "size",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__IntrusionDatum, size),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "flags",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__IntrusionDatum, flags),  // bytes offset in struct
    NULL,  // default value
    sick_safetyscanners2_interfaces__msg__IntrusionDatum__rosidl_typesupport_introspection_c__size_function__IntrusionDatum__flags,  // size() function pointer
    sick_safetyscanners2_interfaces__msg__IntrusionDatum__rosidl_typesupport_introspection_c__get_const_function__IntrusionDatum__flags,  // get_const(index) function pointer
    sick_safetyscanners2_interfaces__msg__IntrusionDatum__rosidl_typesupport_introspection_c__get_function__IntrusionDatum__flags,  // get(index) function pointer
    sick_safetyscanners2_interfaces__msg__IntrusionDatum__rosidl_typesupport_introspection_c__fetch_function__IntrusionDatum__flags,  // fetch(index, &value) function pointer
    sick_safetyscanners2_interfaces__msg__IntrusionDatum__rosidl_typesupport_introspection_c__assign_function__IntrusionDatum__flags,  // assign(index, value) function pointer
    sick_safetyscanners2_interfaces__msg__IntrusionDatum__rosidl_typesupport_introspection_c__resize_function__IntrusionDatum__flags  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers sick_safetyscanners2_interfaces__msg__IntrusionDatum__rosidl_typesupport_introspection_c__IntrusionDatum_message_members = {
  "sick_safetyscanners2_interfaces__msg",  // message namespace
  "IntrusionDatum",  // message name
  2,  // number of fields
  sizeof(sick_safetyscanners2_interfaces__msg__IntrusionDatum),
  sick_safetyscanners2_interfaces__msg__IntrusionDatum__rosidl_typesupport_introspection_c__IntrusionDatum_message_member_array,  // message members
  sick_safetyscanners2_interfaces__msg__IntrusionDatum__rosidl_typesupport_introspection_c__IntrusionDatum_init_function,  // function to initialize message memory (memory has to be allocated)
  sick_safetyscanners2_interfaces__msg__IntrusionDatum__rosidl_typesupport_introspection_c__IntrusionDatum_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t sick_safetyscanners2_interfaces__msg__IntrusionDatum__rosidl_typesupport_introspection_c__IntrusionDatum_message_type_support_handle = {
  0,
  &sick_safetyscanners2_interfaces__msg__IntrusionDatum__rosidl_typesupport_introspection_c__IntrusionDatum_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_sick_safetyscanners2_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, msg, IntrusionDatum)() {
  if (!sick_safetyscanners2_interfaces__msg__IntrusionDatum__rosidl_typesupport_introspection_c__IntrusionDatum_message_type_support_handle.typesupport_identifier) {
    sick_safetyscanners2_interfaces__msg__IntrusionDatum__rosidl_typesupport_introspection_c__IntrusionDatum_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &sick_safetyscanners2_interfaces__msg__IntrusionDatum__rosidl_typesupport_introspection_c__IntrusionDatum_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
