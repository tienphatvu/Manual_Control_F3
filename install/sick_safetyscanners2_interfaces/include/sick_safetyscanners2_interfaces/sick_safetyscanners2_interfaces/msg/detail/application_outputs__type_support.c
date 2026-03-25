// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from sick_safetyscanners2_interfaces:msg/ApplicationOutputs.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "sick_safetyscanners2_interfaces/msg/detail/application_outputs__rosidl_typesupport_introspection_c.h"
#include "sick_safetyscanners2_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "sick_safetyscanners2_interfaces/msg/detail/application_outputs__functions.h"
#include "sick_safetyscanners2_interfaces/msg/detail/application_outputs__struct.h"


// Include directives for member types
// Member `evaluation_path_outputs_eval_out`
// Member `evaluation_path_outputs_is_safe`
// Member `evaluation_path_outputs_is_valid`
// Member `monitoring_case_number_outputs`
// Member `monitoring_case_number_outputs_flags`
// Member `resulting_velocity`
// Member `resulting_velocity_flags`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__ApplicationOutputs_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  sick_safetyscanners2_interfaces__msg__ApplicationOutputs__init(message_memory);
}

void sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__ApplicationOutputs_fini_function(void * message_memory)
{
  sick_safetyscanners2_interfaces__msg__ApplicationOutputs__fini(message_memory);
}

size_t sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__size_function__ApplicationOutputs__evaluation_path_outputs_eval_out(
  const void * untyped_member)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return member->size;
}

const void * sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_const_function__ApplicationOutputs__evaluation_path_outputs_eval_out(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void * sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_function__ApplicationOutputs__evaluation_path_outputs_eval_out(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__fetch_function__ApplicationOutputs__evaluation_path_outputs_eval_out(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_const_function__ApplicationOutputs__evaluation_path_outputs_eval_out(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__assign_function__ApplicationOutputs__evaluation_path_outputs_eval_out(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_function__ApplicationOutputs__evaluation_path_outputs_eval_out(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

bool sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__resize_function__ApplicationOutputs__evaluation_path_outputs_eval_out(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  rosidl_runtime_c__boolean__Sequence__fini(member);
  return rosidl_runtime_c__boolean__Sequence__init(member, size);
}

size_t sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__size_function__ApplicationOutputs__evaluation_path_outputs_is_safe(
  const void * untyped_member)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return member->size;
}

const void * sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_const_function__ApplicationOutputs__evaluation_path_outputs_is_safe(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void * sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_function__ApplicationOutputs__evaluation_path_outputs_is_safe(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__fetch_function__ApplicationOutputs__evaluation_path_outputs_is_safe(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_const_function__ApplicationOutputs__evaluation_path_outputs_is_safe(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__assign_function__ApplicationOutputs__evaluation_path_outputs_is_safe(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_function__ApplicationOutputs__evaluation_path_outputs_is_safe(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

bool sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__resize_function__ApplicationOutputs__evaluation_path_outputs_is_safe(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  rosidl_runtime_c__boolean__Sequence__fini(member);
  return rosidl_runtime_c__boolean__Sequence__init(member, size);
}

size_t sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__size_function__ApplicationOutputs__evaluation_path_outputs_is_valid(
  const void * untyped_member)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return member->size;
}

const void * sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_const_function__ApplicationOutputs__evaluation_path_outputs_is_valid(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void * sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_function__ApplicationOutputs__evaluation_path_outputs_is_valid(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__fetch_function__ApplicationOutputs__evaluation_path_outputs_is_valid(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_const_function__ApplicationOutputs__evaluation_path_outputs_is_valid(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__assign_function__ApplicationOutputs__evaluation_path_outputs_is_valid(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_function__ApplicationOutputs__evaluation_path_outputs_is_valid(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

bool sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__resize_function__ApplicationOutputs__evaluation_path_outputs_is_valid(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  rosidl_runtime_c__boolean__Sequence__fini(member);
  return rosidl_runtime_c__boolean__Sequence__init(member, size);
}

size_t sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__size_function__ApplicationOutputs__monitoring_case_number_outputs(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return member->size;
}

const void * sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_const_function__ApplicationOutputs__monitoring_case_number_outputs(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void * sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_function__ApplicationOutputs__monitoring_case_number_outputs(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__fetch_function__ApplicationOutputs__monitoring_case_number_outputs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint16_t * item =
    ((const uint16_t *)
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_const_function__ApplicationOutputs__monitoring_case_number_outputs(untyped_member, index));
  uint16_t * value =
    (uint16_t *)(untyped_value);
  *value = *item;
}

void sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__assign_function__ApplicationOutputs__monitoring_case_number_outputs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint16_t * item =
    ((uint16_t *)
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_function__ApplicationOutputs__monitoring_case_number_outputs(untyped_member, index));
  const uint16_t * value =
    (const uint16_t *)(untyped_value);
  *item = *value;
}

bool sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__resize_function__ApplicationOutputs__monitoring_case_number_outputs(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  rosidl_runtime_c__uint16__Sequence__fini(member);
  return rosidl_runtime_c__uint16__Sequence__init(member, size);
}

size_t sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__size_function__ApplicationOutputs__monitoring_case_number_outputs_flags(
  const void * untyped_member)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return member->size;
}

const void * sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_const_function__ApplicationOutputs__monitoring_case_number_outputs_flags(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void * sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_function__ApplicationOutputs__monitoring_case_number_outputs_flags(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__fetch_function__ApplicationOutputs__monitoring_case_number_outputs_flags(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_const_function__ApplicationOutputs__monitoring_case_number_outputs_flags(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__assign_function__ApplicationOutputs__monitoring_case_number_outputs_flags(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_function__ApplicationOutputs__monitoring_case_number_outputs_flags(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

bool sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__resize_function__ApplicationOutputs__monitoring_case_number_outputs_flags(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  rosidl_runtime_c__boolean__Sequence__fini(member);
  return rosidl_runtime_c__boolean__Sequence__init(member, size);
}

size_t sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__size_function__ApplicationOutputs__resulting_velocity(
  const void * untyped_member)
{
  const rosidl_runtime_c__int16__Sequence * member =
    (const rosidl_runtime_c__int16__Sequence *)(untyped_member);
  return member->size;
}

const void * sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_const_function__ApplicationOutputs__resulting_velocity(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int16__Sequence * member =
    (const rosidl_runtime_c__int16__Sequence *)(untyped_member);
  return &member->data[index];
}

void * sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_function__ApplicationOutputs__resulting_velocity(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int16__Sequence * member =
    (rosidl_runtime_c__int16__Sequence *)(untyped_member);
  return &member->data[index];
}

void sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__fetch_function__ApplicationOutputs__resulting_velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int16_t * item =
    ((const int16_t *)
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_const_function__ApplicationOutputs__resulting_velocity(untyped_member, index));
  int16_t * value =
    (int16_t *)(untyped_value);
  *value = *item;
}

void sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__assign_function__ApplicationOutputs__resulting_velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int16_t * item =
    ((int16_t *)
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_function__ApplicationOutputs__resulting_velocity(untyped_member, index));
  const int16_t * value =
    (const int16_t *)(untyped_value);
  *item = *value;
}

bool sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__resize_function__ApplicationOutputs__resulting_velocity(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int16__Sequence * member =
    (rosidl_runtime_c__int16__Sequence *)(untyped_member);
  rosidl_runtime_c__int16__Sequence__fini(member);
  return rosidl_runtime_c__int16__Sequence__init(member, size);
}

size_t sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__size_function__ApplicationOutputs__resulting_velocity_flags(
  const void * untyped_member)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return member->size;
}

const void * sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_const_function__ApplicationOutputs__resulting_velocity_flags(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void * sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_function__ApplicationOutputs__resulting_velocity_flags(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__fetch_function__ApplicationOutputs__resulting_velocity_flags(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_const_function__ApplicationOutputs__resulting_velocity_flags(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__assign_function__ApplicationOutputs__resulting_velocity_flags(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_function__ApplicationOutputs__resulting_velocity_flags(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

bool sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__resize_function__ApplicationOutputs__resulting_velocity_flags(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  rosidl_runtime_c__boolean__Sequence__fini(member);
  return rosidl_runtime_c__boolean__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__ApplicationOutputs_message_member_array[22] = {
  {
    "evaluation_path_outputs_eval_out",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__ApplicationOutputs, evaluation_path_outputs_eval_out),  // bytes offset in struct
    NULL,  // default value
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__size_function__ApplicationOutputs__evaluation_path_outputs_eval_out,  // size() function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_const_function__ApplicationOutputs__evaluation_path_outputs_eval_out,  // get_const(index) function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_function__ApplicationOutputs__evaluation_path_outputs_eval_out,  // get(index) function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__fetch_function__ApplicationOutputs__evaluation_path_outputs_eval_out,  // fetch(index, &value) function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__assign_function__ApplicationOutputs__evaluation_path_outputs_eval_out,  // assign(index, value) function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__resize_function__ApplicationOutputs__evaluation_path_outputs_eval_out  // resize(index) function pointer
  },
  {
    "evaluation_path_outputs_is_safe",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__ApplicationOutputs, evaluation_path_outputs_is_safe),  // bytes offset in struct
    NULL,  // default value
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__size_function__ApplicationOutputs__evaluation_path_outputs_is_safe,  // size() function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_const_function__ApplicationOutputs__evaluation_path_outputs_is_safe,  // get_const(index) function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_function__ApplicationOutputs__evaluation_path_outputs_is_safe,  // get(index) function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__fetch_function__ApplicationOutputs__evaluation_path_outputs_is_safe,  // fetch(index, &value) function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__assign_function__ApplicationOutputs__evaluation_path_outputs_is_safe,  // assign(index, value) function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__resize_function__ApplicationOutputs__evaluation_path_outputs_is_safe  // resize(index) function pointer
  },
  {
    "evaluation_path_outputs_is_valid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__ApplicationOutputs, evaluation_path_outputs_is_valid),  // bytes offset in struct
    NULL,  // default value
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__size_function__ApplicationOutputs__evaluation_path_outputs_is_valid,  // size() function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_const_function__ApplicationOutputs__evaluation_path_outputs_is_valid,  // get_const(index) function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_function__ApplicationOutputs__evaluation_path_outputs_is_valid,  // get(index) function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__fetch_function__ApplicationOutputs__evaluation_path_outputs_is_valid,  // fetch(index, &value) function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__assign_function__ApplicationOutputs__evaluation_path_outputs_is_valid,  // assign(index, value) function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__resize_function__ApplicationOutputs__evaluation_path_outputs_is_valid  // resize(index) function pointer
  },
  {
    "monitoring_case_number_outputs",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__ApplicationOutputs, monitoring_case_number_outputs),  // bytes offset in struct
    NULL,  // default value
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__size_function__ApplicationOutputs__monitoring_case_number_outputs,  // size() function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_const_function__ApplicationOutputs__monitoring_case_number_outputs,  // get_const(index) function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_function__ApplicationOutputs__monitoring_case_number_outputs,  // get(index) function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__fetch_function__ApplicationOutputs__monitoring_case_number_outputs,  // fetch(index, &value) function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__assign_function__ApplicationOutputs__monitoring_case_number_outputs,  // assign(index, value) function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__resize_function__ApplicationOutputs__monitoring_case_number_outputs  // resize(index) function pointer
  },
  {
    "monitoring_case_number_outputs_flags",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__ApplicationOutputs, monitoring_case_number_outputs_flags),  // bytes offset in struct
    NULL,  // default value
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__size_function__ApplicationOutputs__monitoring_case_number_outputs_flags,  // size() function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_const_function__ApplicationOutputs__monitoring_case_number_outputs_flags,  // get_const(index) function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_function__ApplicationOutputs__monitoring_case_number_outputs_flags,  // get(index) function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__fetch_function__ApplicationOutputs__monitoring_case_number_outputs_flags,  // fetch(index, &value) function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__assign_function__ApplicationOutputs__monitoring_case_number_outputs_flags,  // assign(index, value) function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__resize_function__ApplicationOutputs__monitoring_case_number_outputs_flags  // resize(index) function pointer
  },
  {
    "sleep_mode_output",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__ApplicationOutputs, sleep_mode_output),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sleep_mode_output_valid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__ApplicationOutputs, sleep_mode_output_valid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_flag_contamination_warning",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__ApplicationOutputs, error_flag_contamination_warning),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_flag_contamination_error",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__ApplicationOutputs, error_flag_contamination_error),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_flag_manipulation_error",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__ApplicationOutputs, error_flag_manipulation_error),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_flag_glare",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__ApplicationOutputs, error_flag_glare),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_flag_reference_contour_intruded",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__ApplicationOutputs, error_flag_reference_contour_intruded),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_flag_critical_error",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__ApplicationOutputs, error_flag_critical_error),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_flags_are_valid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__ApplicationOutputs, error_flags_are_valid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "linear_velocity_outputs_velocity_0",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__ApplicationOutputs, linear_velocity_outputs_velocity_0),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "linear_velocity_outputs_velocity_0_valid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__ApplicationOutputs, linear_velocity_outputs_velocity_0_valid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "linear_velocity_outputs_velocity_0_transmitted_safely",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__ApplicationOutputs, linear_velocity_outputs_velocity_0_transmitted_safely),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "linear_velocity_outputs_velocity_1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__ApplicationOutputs, linear_velocity_outputs_velocity_1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "linear_velocity_outputs_velocity_1_valid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__ApplicationOutputs, linear_velocity_outputs_velocity_1_valid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "linear_velocity_outputs_velocity_1_transmitted_safely",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__ApplicationOutputs, linear_velocity_outputs_velocity_1_transmitted_safely),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "resulting_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__ApplicationOutputs, resulting_velocity),  // bytes offset in struct
    NULL,  // default value
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__size_function__ApplicationOutputs__resulting_velocity,  // size() function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_const_function__ApplicationOutputs__resulting_velocity,  // get_const(index) function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_function__ApplicationOutputs__resulting_velocity,  // get(index) function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__fetch_function__ApplicationOutputs__resulting_velocity,  // fetch(index, &value) function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__assign_function__ApplicationOutputs__resulting_velocity,  // assign(index, value) function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__resize_function__ApplicationOutputs__resulting_velocity  // resize(index) function pointer
  },
  {
    "resulting_velocity_flags",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__msg__ApplicationOutputs, resulting_velocity_flags),  // bytes offset in struct
    NULL,  // default value
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__size_function__ApplicationOutputs__resulting_velocity_flags,  // size() function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_const_function__ApplicationOutputs__resulting_velocity_flags,  // get_const(index) function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__get_function__ApplicationOutputs__resulting_velocity_flags,  // get(index) function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__fetch_function__ApplicationOutputs__resulting_velocity_flags,  // fetch(index, &value) function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__assign_function__ApplicationOutputs__resulting_velocity_flags,  // assign(index, value) function pointer
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__resize_function__ApplicationOutputs__resulting_velocity_flags  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__ApplicationOutputs_message_members = {
  "sick_safetyscanners2_interfaces__msg",  // message namespace
  "ApplicationOutputs",  // message name
  22,  // number of fields
  sizeof(sick_safetyscanners2_interfaces__msg__ApplicationOutputs),
  sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__ApplicationOutputs_message_member_array,  // message members
  sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__ApplicationOutputs_init_function,  // function to initialize message memory (memory has to be allocated)
  sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__ApplicationOutputs_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__ApplicationOutputs_message_type_support_handle = {
  0,
  &sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__ApplicationOutputs_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_sick_safetyscanners2_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, msg, ApplicationOutputs)() {
  if (!sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__ApplicationOutputs_message_type_support_handle.typesupport_identifier) {
    sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__ApplicationOutputs_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &sick_safetyscanners2_interfaces__msg__ApplicationOutputs__rosidl_typesupport_introspection_c__ApplicationOutputs_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
