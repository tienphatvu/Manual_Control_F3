// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from sick_safetyscanners2_interfaces:msg/IntrusionData.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "sick_safetyscanners2_interfaces/msg/detail/intrusion_data__struct.hpp"
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

void IntrusionData_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) sick_safetyscanners2_interfaces::msg::IntrusionData(_init);
}

void IntrusionData_fini_function(void * message_memory)
{
  auto typed_message = static_cast<sick_safetyscanners2_interfaces::msg::IntrusionData *>(message_memory);
  typed_message->~IntrusionData();
}

size_t size_function__IntrusionData__data(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<sick_safetyscanners2_interfaces::msg::IntrusionDatum> *>(untyped_member);
  return member->size();
}

const void * get_const_function__IntrusionData__data(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<sick_safetyscanners2_interfaces::msg::IntrusionDatum> *>(untyped_member);
  return &member[index];
}

void * get_function__IntrusionData__data(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<sick_safetyscanners2_interfaces::msg::IntrusionDatum> *>(untyped_member);
  return &member[index];
}

void fetch_function__IntrusionData__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const sick_safetyscanners2_interfaces::msg::IntrusionDatum *>(
    get_const_function__IntrusionData__data(untyped_member, index));
  auto & value = *reinterpret_cast<sick_safetyscanners2_interfaces::msg::IntrusionDatum *>(untyped_value);
  value = item;
}

void assign_function__IntrusionData__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<sick_safetyscanners2_interfaces::msg::IntrusionDatum *>(
    get_function__IntrusionData__data(untyped_member, index));
  const auto & value = *reinterpret_cast<const sick_safetyscanners2_interfaces::msg::IntrusionDatum *>(untyped_value);
  item = value;
}

void resize_function__IntrusionData__data(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<sick_safetyscanners2_interfaces::msg::IntrusionDatum> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember IntrusionData_message_member_array[1] = {
  {
    "data",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<sick_safetyscanners2_interfaces::msg::IntrusionDatum>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces::msg::IntrusionData, data),  // bytes offset in struct
    nullptr,  // default value
    size_function__IntrusionData__data,  // size() function pointer
    get_const_function__IntrusionData__data,  // get_const(index) function pointer
    get_function__IntrusionData__data,  // get(index) function pointer
    fetch_function__IntrusionData__data,  // fetch(index, &value) function pointer
    assign_function__IntrusionData__data,  // assign(index, value) function pointer
    resize_function__IntrusionData__data  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers IntrusionData_message_members = {
  "sick_safetyscanners2_interfaces::msg",  // message namespace
  "IntrusionData",  // message name
  1,  // number of fields
  sizeof(sick_safetyscanners2_interfaces::msg::IntrusionData),
  IntrusionData_message_member_array,  // message members
  IntrusionData_init_function,  // function to initialize message memory (memory has to be allocated)
  IntrusionData_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t IntrusionData_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &IntrusionData_message_members,
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
get_message_type_support_handle<sick_safetyscanners2_interfaces::msg::IntrusionData>()
{
  return &::sick_safetyscanners2_interfaces::msg::rosidl_typesupport_introspection_cpp::IntrusionData_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, sick_safetyscanners2_interfaces, msg, IntrusionData)() {
  return &::sick_safetyscanners2_interfaces::msg::rosidl_typesupport_introspection_cpp::IntrusionData_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
