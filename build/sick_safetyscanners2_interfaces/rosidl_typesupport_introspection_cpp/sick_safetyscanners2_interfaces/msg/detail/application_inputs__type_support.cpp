// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from sick_safetyscanners2_interfaces:msg/ApplicationInputs.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "sick_safetyscanners2_interfaces/msg/detail/application_inputs__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace sick_safetyscanners2_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ApplicationInputs_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) sick_safetyscanners2_interfaces::msg::ApplicationInputs(_init);
}

void ApplicationInputs_fini_function(void * message_memory)
{
  auto typed_message = static_cast<sick_safetyscanners2_interfaces::msg::ApplicationInputs *>(message_memory);
  typed_message->~ApplicationInputs();
}

size_t size_function__ApplicationInputs__unsafe_inputs_input_sources(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<bool> *>(untyped_member);
  return member->size();
}

void fetch_function__ApplicationInputs__unsafe_inputs_input_sources(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & member = *reinterpret_cast<const std::vector<bool> *>(untyped_member);
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = member[index];
}

void assign_function__ApplicationInputs__unsafe_inputs_input_sources(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & member = *reinterpret_cast<std::vector<bool> *>(untyped_member);
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  member[index] = value;
}

void resize_function__ApplicationInputs__unsafe_inputs_input_sources(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<bool> *>(untyped_member);
  member->resize(size);
}

size_t size_function__ApplicationInputs__unsafe_inputs_flags(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<bool> *>(untyped_member);
  return member->size();
}

void fetch_function__ApplicationInputs__unsafe_inputs_flags(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & member = *reinterpret_cast<const std::vector<bool> *>(untyped_member);
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = member[index];
}

void assign_function__ApplicationInputs__unsafe_inputs_flags(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & member = *reinterpret_cast<std::vector<bool> *>(untyped_member);
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  member[index] = value;
}

void resize_function__ApplicationInputs__unsafe_inputs_flags(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<bool> *>(untyped_member);
  member->resize(size);
}

size_t size_function__ApplicationInputs__monitoring_case_number_inputs(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ApplicationInputs__monitoring_case_number_inputs(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void * get_function__ApplicationInputs__monitoring_case_number_inputs(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__ApplicationInputs__monitoring_case_number_inputs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint16_t *>(
    get_const_function__ApplicationInputs__monitoring_case_number_inputs(untyped_member, index));
  auto & value = *reinterpret_cast<uint16_t *>(untyped_value);
  value = item;
}

void assign_function__ApplicationInputs__monitoring_case_number_inputs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint16_t *>(
    get_function__ApplicationInputs__monitoring_case_number_inputs(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint16_t *>(untyped_value);
  item = value;
}

void resize_function__ApplicationInputs__monitoring_case_number_inputs(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__ApplicationInputs__monitoring_case_number_inputs_flags(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<bool> *>(untyped_member);
  return member->size();
}

void fetch_function__ApplicationInputs__monitoring_case_number_inputs_flags(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & member = *reinterpret_cast<const std::vector<bool> *>(untyped_member);
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = member[index];
}

void assign_function__ApplicationInputs__monitoring_case_number_inputs_flags(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & member = *reinterpret_cast<std::vector<bool> *>(untyped_member);
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  member[index] = value;
}

void resize_function__ApplicationInputs__monitoring_case_number_inputs_flags(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<bool> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ApplicationInputs_message_member_array[11] = {
  {
    "unsafe_inputs_input_sources",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces::msg::ApplicationInputs, unsafe_inputs_input_sources),  // bytes offset in struct
    nullptr,  // default value
    size_function__ApplicationInputs__unsafe_inputs_input_sources,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    fetch_function__ApplicationInputs__unsafe_inputs_input_sources,  // fetch(index, &value) function pointer
    assign_function__ApplicationInputs__unsafe_inputs_input_sources,  // assign(index, value) function pointer
    resize_function__ApplicationInputs__unsafe_inputs_input_sources  // resize(index) function pointer
  },
  {
    "unsafe_inputs_flags",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces::msg::ApplicationInputs, unsafe_inputs_flags),  // bytes offset in struct
    nullptr,  // default value
    size_function__ApplicationInputs__unsafe_inputs_flags,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    fetch_function__ApplicationInputs__unsafe_inputs_flags,  // fetch(index, &value) function pointer
    assign_function__ApplicationInputs__unsafe_inputs_flags,  // assign(index, value) function pointer
    resize_function__ApplicationInputs__unsafe_inputs_flags  // resize(index) function pointer
  },
  {
    "monitoring_case_number_inputs",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces::msg::ApplicationInputs, monitoring_case_number_inputs),  // bytes offset in struct
    nullptr,  // default value
    size_function__ApplicationInputs__monitoring_case_number_inputs,  // size() function pointer
    get_const_function__ApplicationInputs__monitoring_case_number_inputs,  // get_const(index) function pointer
    get_function__ApplicationInputs__monitoring_case_number_inputs,  // get(index) function pointer
    fetch_function__ApplicationInputs__monitoring_case_number_inputs,  // fetch(index, &value) function pointer
    assign_function__ApplicationInputs__monitoring_case_number_inputs,  // assign(index, value) function pointer
    resize_function__ApplicationInputs__monitoring_case_number_inputs  // resize(index) function pointer
  },
  {
    "monitoring_case_number_inputs_flags",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces::msg::ApplicationInputs, monitoring_case_number_inputs_flags),  // bytes offset in struct
    nullptr,  // default value
    size_function__ApplicationInputs__monitoring_case_number_inputs_flags,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    fetch_function__ApplicationInputs__monitoring_case_number_inputs_flags,  // fetch(index, &value) function pointer
    assign_function__ApplicationInputs__monitoring_case_number_inputs_flags,  // assign(index, value) function pointer
    resize_function__ApplicationInputs__monitoring_case_number_inputs_flags  // resize(index) function pointer
  },
  {
    "linear_velocity_inputs_velocity_0",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces::msg::ApplicationInputs, linear_velocity_inputs_velocity_0),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "linear_velocity_inputs_velocity_0_valid",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces::msg::ApplicationInputs, linear_velocity_inputs_velocity_0_valid),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "linear_velocity_inputs_velocity_0_transmitted_safely",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces::msg::ApplicationInputs, linear_velocity_inputs_velocity_0_transmitted_safely),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "linear_velocity_inputs_velocity_1",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces::msg::ApplicationInputs, linear_velocity_inputs_velocity_1),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "linear_velocity_inputs_velocity_1_valid",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces::msg::ApplicationInputs, linear_velocity_inputs_velocity_1_valid),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "linear_velocity_inputs_velocity_1_transmitted_safely",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces::msg::ApplicationInputs, linear_velocity_inputs_velocity_1_transmitted_safely),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "sleep_mode_input",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces::msg::ApplicationInputs, sleep_mode_input),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ApplicationInputs_message_members = {
  "sick_safetyscanners2_interfaces::msg",  // message namespace
  "ApplicationInputs",  // message name
  11,  // number of fields
  sizeof(sick_safetyscanners2_interfaces::msg::ApplicationInputs),
  ApplicationInputs_message_member_array,  // message members
  ApplicationInputs_init_function,  // function to initialize message memory (memory has to be allocated)
  ApplicationInputs_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ApplicationInputs_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ApplicationInputs_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace sick_safetyscanners2_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<sick_safetyscanners2_interfaces::msg::ApplicationInputs>()
{
  return &::sick_safetyscanners2_interfaces::msg::rosidl_typesupport_introspection_cpp::ApplicationInputs_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, sick_safetyscanners2_interfaces, msg, ApplicationInputs)() {
  return &::sick_safetyscanners2_interfaces::msg::rosidl_typesupport_introspection_cpp::ApplicationInputs_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
