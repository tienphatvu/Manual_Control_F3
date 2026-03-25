// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from sick_safetyscanners2_interfaces:msg/OutputPaths.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "sick_safetyscanners2_interfaces/msg/detail/output_paths__struct.hpp"
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

void OutputPaths_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) sick_safetyscanners2_interfaces::msg::OutputPaths(_init);
}

void OutputPaths_fini_function(void * message_memory)
{
  auto typed_message = static_cast<sick_safetyscanners2_interfaces::msg::OutputPaths *>(message_memory);
  typed_message->~OutputPaths();
}

size_t size_function__OutputPaths__status(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<bool> *>(untyped_member);
  return member->size();
}

void fetch_function__OutputPaths__status(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & member = *reinterpret_cast<const std::vector<bool> *>(untyped_member);
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = member[index];
}

void assign_function__OutputPaths__status(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & member = *reinterpret_cast<std::vector<bool> *>(untyped_member);
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  member[index] = value;
}

void resize_function__OutputPaths__status(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<bool> *>(untyped_member);
  member->resize(size);
}

size_t size_function__OutputPaths__is_safe(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<bool> *>(untyped_member);
  return member->size();
}

void fetch_function__OutputPaths__is_safe(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & member = *reinterpret_cast<const std::vector<bool> *>(untyped_member);
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = member[index];
}

void assign_function__OutputPaths__is_safe(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & member = *reinterpret_cast<std::vector<bool> *>(untyped_member);
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  member[index] = value;
}

void resize_function__OutputPaths__is_safe(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<bool> *>(untyped_member);
  member->resize(size);
}

size_t size_function__OutputPaths__is_valid(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<bool> *>(untyped_member);
  return member->size();
}

void fetch_function__OutputPaths__is_valid(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & member = *reinterpret_cast<const std::vector<bool> *>(untyped_member);
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = member[index];
}

void assign_function__OutputPaths__is_valid(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & member = *reinterpret_cast<std::vector<bool> *>(untyped_member);
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  member[index] = value;
}

void resize_function__OutputPaths__is_valid(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<bool> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember OutputPaths_message_member_array[4] = {
  {
    "status",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces::msg::OutputPaths, status),  // bytes offset in struct
    nullptr,  // default value
    size_function__OutputPaths__status,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    fetch_function__OutputPaths__status,  // fetch(index, &value) function pointer
    assign_function__OutputPaths__status,  // assign(index, value) function pointer
    resize_function__OutputPaths__status  // resize(index) function pointer
  },
  {
    "is_safe",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces::msg::OutputPaths, is_safe),  // bytes offset in struct
    nullptr,  // default value
    size_function__OutputPaths__is_safe,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    fetch_function__OutputPaths__is_safe,  // fetch(index, &value) function pointer
    assign_function__OutputPaths__is_safe,  // assign(index, value) function pointer
    resize_function__OutputPaths__is_safe  // resize(index) function pointer
  },
  {
    "is_valid",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces::msg::OutputPaths, is_valid),  // bytes offset in struct
    nullptr,  // default value
    size_function__OutputPaths__is_valid,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    fetch_function__OutputPaths__is_valid,  // fetch(index, &value) function pointer
    assign_function__OutputPaths__is_valid,  // assign(index, value) function pointer
    resize_function__OutputPaths__is_valid  // resize(index) function pointer
  },
  {
    "active_monitoring_case",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces::msg::OutputPaths, active_monitoring_case),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers OutputPaths_message_members = {
  "sick_safetyscanners2_interfaces::msg",  // message namespace
  "OutputPaths",  // message name
  4,  // number of fields
  sizeof(sick_safetyscanners2_interfaces::msg::OutputPaths),
  OutputPaths_message_member_array,  // message members
  OutputPaths_init_function,  // function to initialize message memory (memory has to be allocated)
  OutputPaths_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t OutputPaths_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &OutputPaths_message_members,
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
get_message_type_support_handle<sick_safetyscanners2_interfaces::msg::OutputPaths>()
{
  return &::sick_safetyscanners2_interfaces::msg::rosidl_typesupport_introspection_cpp::OutputPaths_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, sick_safetyscanners2_interfaces, msg, OutputPaths)() {
  return &::sick_safetyscanners2_interfaces::msg::rosidl_typesupport_introspection_cpp::OutputPaths_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
