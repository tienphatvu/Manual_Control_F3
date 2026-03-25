// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from sick_safetyscanners2_interfaces:msg/ExtendedLaserScan.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "sick_safetyscanners2_interfaces/msg/detail/extended_laser_scan__rosidl_typesupport_introspection_c.h"
#include "sick_safetyscanners2_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "sick_safetyscanners2_interfaces/msg/detail/extended_laser_scan__functions.h"
#include "sick_safetyscanners2_interfaces/msg/detail/extended_laser_scan__struct.h"


// Include directives for member types
// Member `laser_scan`
#include "sensor_msgs/msg/laser_scan.h"
// Member `laser_scan`
#include "sensor_msgs/msg/detail/laser_scan__rosidl_typesupport_introspection_c.h"
// Member `reflektor_status`
// Member `reflektor_median`
// Member `intrusion`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__ExtendedLaserScan_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__init(message_memory);
}

void sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__ExtendedLaserScan_fini_function(void * message_memory)
{
  sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__fini(message_memory);
}

size_t sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__size_function__ExtendedLaserScan__reflektor_status(
  const void * untyped_member)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return member->size;
}

const void * sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__get_const_function__ExtendedLaserScan__reflektor_status(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void * sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__get_function__ExtendedLaserScan__reflektor_status(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__fetch_function__ExtendedLaserScan__reflektor_status(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__get_const_function__ExtendedLaserScan__reflektor_status(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__assign_function__ExtendedLaserScan__reflektor_status(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__get_function__ExtendedLaserScan__reflektor_status(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

bool sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__resize_function__ExtendedLaserScan__reflektor_status(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  rosidl_runtime_c__boolean__Sequence__fini(member);
  return rosidl_runtime_c__boolean__Sequence__init(member, size);
}

size_t sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__size_function__ExtendedLaserScan__reflektor_median(
  const void * untyped_member)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return member->size;
}

const void * sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__get_const_function__ExtendedLaserScan__reflektor_median(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void * sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__get_function__ExtendedLaserScan__reflektor_median(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__fetch_function__ExtendedLaserScan__reflektor_median(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__get_const_function__ExtendedLaserScan__reflektor_median(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__assign_function__ExtendedLaserScan__reflektor_median(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__get_function__ExtendedLaserScan__reflektor_median(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

bool sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__resize_function__ExtendedLaserScan__reflektor_median(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  rosidl_runtime_c__boolean__Sequence__fini(member);
  return rosidl_runtime_c__boolean__Sequence__init(member, size);
}

size_t sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__size_function__ExtendedLaserScan__intrusion(
  const void * untyped_member)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return member->size;
}

const void * sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__get_const_function__ExtendedLaserScan__intrusion(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void * sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__get_function__ExtendedLaserScan__intrusion(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__fetch_function__ExtendedLaserScan__intrusion(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__get_const_function__ExtendedLaserScan__intrusion(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__assign_function__ExtendedLaserScan__intrusion(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__get_function__ExtendedLaserScan__intrusion(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

bool sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__resize_function__ExtendedLaserScan__intrusion(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  rosidl_runtime_c__boolean__Sequence__fini(member);
  return rosidl_runtime_c__boolean__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__ExtendedLaserScan_message_member_array[4] = {
  {
    "laser_scan",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__ExtendedLaserScan, laser_scan),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "reflektor_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__ExtendedLaserScan, reflektor_status),  // bytes offset in struct
    NULL,  // default value
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__size_function__ExtendedLaserScan__reflektor_status,  // size() function pointer
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__get_const_function__ExtendedLaserScan__reflektor_status,  // get_const(index) function pointer
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__get_function__ExtendedLaserScan__reflektor_status,  // get(index) function pointer
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__fetch_function__ExtendedLaserScan__reflektor_status,  // fetch(index, &value) function pointer
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__assign_function__ExtendedLaserScan__reflektor_status,  // assign(index, value) function pointer
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__resize_function__ExtendedLaserScan__reflektor_status  // resize(index) function pointer
  },
  {
    "reflektor_median",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__ExtendedLaserScan, reflektor_median),  // bytes offset in struct
    NULL,  // default value
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__size_function__ExtendedLaserScan__reflektor_median,  // size() function pointer
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__get_const_function__ExtendedLaserScan__reflektor_median,  // get_const(index) function pointer
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__get_function__ExtendedLaserScan__reflektor_median,  // get(index) function pointer
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__fetch_function__ExtendedLaserScan__reflektor_median,  // fetch(index, &value) function pointer
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__assign_function__ExtendedLaserScan__reflektor_median,  // assign(index, value) function pointer
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__resize_function__ExtendedLaserScan__reflektor_median  // resize(index) function pointer
  },
  {
    "intrusion",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__ExtendedLaserScan, intrusion),  // bytes offset in struct
    NULL,  // default value
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__size_function__ExtendedLaserScan__intrusion,  // size() function pointer
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__get_const_function__ExtendedLaserScan__intrusion,  // get_const(index) function pointer
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__get_function__ExtendedLaserScan__intrusion,  // get(index) function pointer
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__fetch_function__ExtendedLaserScan__intrusion,  // fetch(index, &value) function pointer
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__assign_function__ExtendedLaserScan__intrusion,  // assign(index, value) function pointer
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__resize_function__ExtendedLaserScan__intrusion  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__ExtendedLaserScan_message_members = {
  "sick_safetyscanners2_interfaces__msg",  // message namespace
  "ExtendedLaserScan",  // message name
  4,  // number of fields
  sizeof(sick_safetyscanners2_interfaces__msg__ExtendedLaserScan),
  sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__ExtendedLaserScan_message_member_array,  // message members
  sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__ExtendedLaserScan_init_function,  // function to initialize message memory (memory has to be allocated)
  sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__ExtendedLaserScan_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__ExtendedLaserScan_message_type_support_handle = {
  0,
  &sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__ExtendedLaserScan_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_sick_safetyscanners2_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, msg, ExtendedLaserScan)() {
  sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__ExtendedLaserScan_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, LaserScan)();
  if (!sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__ExtendedLaserScan_message_type_support_handle.typesupport_identifier) {
    sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__ExtendedLaserScan_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__rosidl_typesupport_introspection_c__ExtendedLaserScan_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
