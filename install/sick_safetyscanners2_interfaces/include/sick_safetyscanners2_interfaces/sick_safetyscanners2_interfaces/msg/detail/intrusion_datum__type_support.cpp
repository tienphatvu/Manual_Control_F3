// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from sick_safetyscanners2_interfaces:msg/IntrusionDatum.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "sick_safetyscanners2_interfaces/msg/detail/intrusion_datum__struct.hpp"
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

void IntrusionDatum_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) sick_safetyscanners2_interfaces::msg::IntrusionDatum(_init);
}

void IntrusionDatum_fini_function(void * message_memory)
{
  auto typed_message = static_cast<sick_safetyscanners2_interfaces::msg::IntrusionDatum *>(message_memory);
  typed_message->~IntrusionDatum();
}

size_t size_function__IntrusionDatum__flags(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<bool> *>(untyped_member);
  return member->size();
}

void fetch_function__IntrusionDatum__flags(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & member = *reinterpret_cast<const std::vector<bool> *>(untyped_member);
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = member[index];
}

void assign_function__IntrusionDatum__flags(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & member = *reinterpret_cast<std::vector<bool> *>(untyped_member);
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  member[index] = value;
}

void resize_function__IntrusionDatum__flags(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<bool> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember IntrusionDatum_message_member_array[2] = {
  {
    "size",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces::msg::IntrusionDatum, size),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "flags",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces::msg::IntrusionDatum, flags),  // bytes offset in struct
    nullptr,  // default value
    size_function__IntrusionDatum__flags,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    fetch_function__IntrusionDatum__flags,  // fetch(index, &value) function pointer
    assign_function__IntrusionDatum__flags,  // assign(index, value) function pointer
    resize_function__IntrusionDatum__flags  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers IntrusionDatum_message_members = {
  "sick_safetyscanners2_interfaces::msg",  // message namespace
  "IntrusionDatum",  // message name
  2,  // number of fields
  sizeof(sick_safetyscanners2_interfaces::msg::IntrusionDatum),
  IntrusionDatum_message_member_array,  // message members
  IntrusionDatum_init_function,  // function to initialize message memory (memory has to be allocated)
  IntrusionDatum_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t IntrusionDatum_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &IntrusionDatum_message_members,
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
get_message_type_support_handle<sick_safetyscanners2_interfaces::msg::IntrusionDatum>()
{
  return &::sick_safetyscanners2_interfaces::msg::rosidl_typesupport_introspection_cpp::IntrusionDatum_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, sick_safetyscanners2_interfaces, msg, IntrusionDatum)() {
  return &::sick_safetyscanners2_interfaces::msg::rosidl_typesupport_introspection_cpp::IntrusionDatum_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
