// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from sick_safetyscanners2_interfaces:msg/IntrusionData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "sick_safetyscanners2_interfaces/msg/detail/intrusion_data__rosidl_typesupport_introspection_c.h"
#include "sick_safetyscanners2_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "sick_safetyscanners2_interfaces/msg/detail/intrusion_data__functions.h"
#include "sick_safetyscanners2_interfaces/msg/detail/intrusion_data__struct.h"


// Include directives for member types
// Member `data`
#include "sick_safetyscanners2_interfaces/msg/intrusion_datum.h"
// Member `data`
#include "sick_safetyscanners2_interfaces/msg/detail/intrusion_datum__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void sick_safetyscanners2_interfaces__msg__IntrusionData__rosidl_typesupport_introspection_c__IntrusionData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  sick_safetyscanners2_interfaces__msg__IntrusionData__init(message_memory);
}

void sick_safetyscanners2_interfaces__msg__IntrusionData__rosidl_typesupport_introspection_c__IntrusionData_fini_function(void * message_memory)
{
  sick_safetyscanners2_interfaces__msg__IntrusionData__fini(message_memory);
}

size_t sick_safetyscanners2_interfaces__msg__IntrusionData__rosidl_typesupport_introspection_c__size_function__IntrusionData__data(
  const void * untyped_member)
{
  const sick_safetyscanners2_interfaces__msg__IntrusionDatum__Sequence * member =
    (const sick_safetyscanners2_interfaces__msg__IntrusionDatum__Sequence *)(untyped_member);
  return member->size;
}

const void * sick_safetyscanners2_interfaces__msg__IntrusionData__rosidl_typesupport_introspection_c__get_const_function__IntrusionData__data(
  const void * untyped_member, size_t index)
{
  const sick_safetyscanners2_interfaces__msg__IntrusionDatum__Sequence * member =
    (const sick_safetyscanners2_interfaces__msg__IntrusionDatum__Sequence *)(untyped_member);
  return &member->data[index];
}

void * sick_safetyscanners2_interfaces__msg__IntrusionData__rosidl_typesupport_introspection_c__get_function__IntrusionData__data(
  void * untyped_member, size_t index)
{
  sick_safetyscanners2_interfaces__msg__IntrusionDatum__Sequence * member =
    (sick_safetyscanners2_interfaces__msg__IntrusionDatum__Sequence *)(untyped_member);
  return &member->data[index];
}

void sick_safetyscanners2_interfaces__msg__IntrusionData__rosidl_typesupport_introspection_c__fetch_function__IntrusionData__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const sick_safetyscanners2_interfaces__msg__IntrusionDatum * item =
    ((const sick_safetyscanners2_interfaces__msg__IntrusionDatum *)
    sick_safetyscanners2_interfaces__msg__IntrusionData__rosidl_typesupport_introspection_c__get_const_function__IntrusionData__data(untyped_member, index));
  sick_safetyscanners2_interfaces__msg__IntrusionDatum * value =
    (sick_safetyscanners2_interfaces__msg__IntrusionDatum *)(untyped_value);
  *value = *item;
}

void sick_safetyscanners2_interfaces__msg__IntrusionData__rosidl_typesupport_introspection_c__assign_function__IntrusionData__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  sick_safetyscanners2_interfaces__msg__IntrusionDatum * item =
    ((sick_safetyscanners2_interfaces__msg__IntrusionDatum *)
    sick_safetyscanners2_interfaces__msg__IntrusionData__rosidl_typesupport_introspection_c__get_function__IntrusionData__data(untyped_member, index));
  const sick_safetyscanners2_interfaces__msg__IntrusionDatum * value =
    (const sick_safetyscanners2_interfaces__msg__IntrusionDatum *)(untyped_value);
  *item = *value;
}

bool sick_safetyscanners2_interfaces__msg__IntrusionData__rosidl_typesupport_introspection_c__resize_function__IntrusionData__data(
  void * untyped_member, size_t size)
{
  sick_safetyscanners2_interfaces__msg__IntrusionDatum__Sequence * member =
    (sick_safetyscanners2_interfaces__msg__IntrusionDatum__Sequence *)(untyped_member);
  sick_safetyscanners2_interfaces__msg__IntrusionDatum__Sequence__fini(member);
  return sick_safetyscanners2_interfaces__msg__IntrusionDatum__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember sick_safetyscanners2_interfaces__msg__IntrusionData__rosidl_typesupport_introspection_c__IntrusionData_message_member_array[1] = {
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__IntrusionData, data),  // bytes offset in struct
    NULL,  // default value
    sick_safetyscanners2_interfaces__msg__IntrusionData__rosidl_typesupport_introspection_c__size_function__IntrusionData__data,  // size() function pointer
    sick_safetyscanners2_interfaces__msg__IntrusionData__rosidl_typesupport_introspection_c__get_const_function__IntrusionData__data,  // get_const(index) function pointer
    sick_safetyscanners2_interfaces__msg__IntrusionData__rosidl_typesupport_introspection_c__get_function__IntrusionData__data,  // get(index) function pointer
    sick_safetyscanners2_interfaces__msg__IntrusionData__rosidl_typesupport_introspection_c__fetch_function__IntrusionData__data,  // fetch(index, &value) function pointer
    sick_safetyscanners2_interfaces__msg__IntrusionData__rosidl_typesupport_introspection_c__assign_function__IntrusionData__data,  // assign(index, value) function pointer
    sick_safetyscanners2_interfaces__msg__IntrusionData__rosidl_typesupport_introspection_c__resize_function__IntrusionData__data  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers sick_safetyscanners2_interfaces__msg__IntrusionData__rosidl_typesupport_introspection_c__IntrusionData_message_members = {
  "sick_safetyscanners2_interfaces__msg",  // message namespace
  "IntrusionData",  // message name
  1,  // number of fields
  sizeof(sick_safetyscanners2_interfaces__msg__IntrusionData),
  sick_safetyscanners2_interfaces__msg__IntrusionData__rosidl_typesupport_introspection_c__IntrusionData_message_member_array,  // message members
  sick_safetyscanners2_interfaces__msg__IntrusionData__rosidl_typesupport_introspection_c__IntrusionData_init_function,  // function to initialize message memory (memory has to be allocated)
  sick_safetyscanners2_interfaces__msg__IntrusionData__rosidl_typesupport_introspection_c__IntrusionData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t sick_safetyscanners2_interfaces__msg__IntrusionData__rosidl_typesupport_introspection_c__IntrusionData_message_type_support_handle = {
  0,
  &sick_safetyscanners2_interfaces__msg__IntrusionData__rosidl_typesupport_introspection_c__IntrusionData_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_sick_safetyscanners2_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, msg, IntrusionData)() {
  sick_safetyscanners2_interfaces__msg__IntrusionData__rosidl_typesupport_introspection_c__IntrusionData_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, msg, IntrusionDatum)();
  if (!sick_safetyscanners2_interfaces__msg__IntrusionData__rosidl_typesupport_introspection_c__IntrusionData_message_type_support_handle.typesupport_identifier) {
    sick_safetyscanners2_interfaces__msg__IntrusionData__rosidl_typesupport_introspection_c__IntrusionData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &sick_safetyscanners2_interfaces__msg__IntrusionData__rosidl_typesupport_introspection_c__IntrusionData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
