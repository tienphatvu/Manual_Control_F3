// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from sick_safetyscanners2_interfaces:msg/ExtendedLaserScan.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "sick_safetyscanners2_interfaces/msg/detail/extended_laser_scan__struct.hpp"
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

void ExtendedLaserScan_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) sick_safetyscanners2_interfaces::msg::ExtendedLaserScan(_init);
}

void ExtendedLaserScan_fini_function(void * message_memory)
{
  auto typed_message = static_cast<sick_safetyscanners2_interfaces::msg::ExtendedLaserScan *>(message_memory);
  typed_message->~ExtendedLaserScan();
}

size_t size_function__ExtendedLaserScan__reflektor_status(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<bool> *>(untyped_member);
  return member->size();
}

void fetch_function__ExtendedLaserScan__reflektor_status(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & member = *reinterpret_cast<const std::vector<bool> *>(untyped_member);
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = member[index];
}

void assign_function__ExtendedLaserScan__reflektor_status(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & member = *reinterpret_cast<std::vector<bool> *>(untyped_member);
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  member[index] = value;
}

void resize_function__ExtendedLaserScan__reflektor_status(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<bool> *>(untyped_member);
  member->resize(size);
}

size_t size_function__ExtendedLaserScan__reflektor_median(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<bool> *>(untyped_member);
  return member->size();
}

void fetch_function__ExtendedLaserScan__reflektor_median(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & member = *reinterpret_cast<const std::vector<bool> *>(untyped_member);
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = member[index];
}

void assign_function__ExtendedLaserScan__reflektor_median(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & member = *reinterpret_cast<std::vector<bool> *>(untyped_member);
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  member[index] = value;
}

void resize_function__ExtendedLaserScan__reflektor_median(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<bool> *>(untyped_member);
  member->resize(size);
}

size_t size_function__ExtendedLaserScan__intrusion(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<bool> *>(untyped_member);
  return member->size();
}

void fetch_function__ExtendedLaserScan__intrusion(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & member = *reinterpret_cast<const std::vector<bool> *>(untyped_member);
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = member[index];
}

void assign_function__ExtendedLaserScan__intrusion(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & member = *reinterpret_cast<std::vector<bool> *>(untyped_member);
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  member[index] = value;
}

void resize_function__ExtendedLaserScan__intrusion(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<bool> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ExtendedLaserScan_message_member_array[4] = {
  {
    "laser_scan",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<sensor_msgs::msg::LaserScan>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces::msg::ExtendedLaserScan, laser_scan),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "reflektor_status",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces::msg::ExtendedLaserScan, reflektor_status),  // bytes offset in struct
    nullptr,  // default value
    size_function__ExtendedLaserScan__reflektor_status,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    fetch_function__ExtendedLaserScan__reflektor_status,  // fetch(index, &value) function pointer
    assign_function__ExtendedLaserScan__reflektor_status,  // assign(index, value) function pointer
    resize_function__ExtendedLaserScan__reflektor_status  // resize(index) function pointer
  },
  {
    "reflektor_median",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces::msg::ExtendedLaserScan, reflektor_median),  // bytes offset in struct
    nullptr,  // default value
    size_function__ExtendedLaserScan__reflektor_median,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    fetch_function__ExtendedLaserScan__reflektor_median,  // fetch(index, &value) function pointer
    assign_function__ExtendedLaserScan__reflektor_median,  // assign(index, value) function pointer
    resize_function__ExtendedLaserScan__reflektor_median  // resize(index) function pointer
  },
  {
    "intrusion",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces::msg::ExtendedLaserScan, intrusion),  // bytes offset in struct
    nullptr,  // default value
    size_function__ExtendedLaserScan__intrusion,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    fetch_function__ExtendedLaserScan__intrusion,  // fetch(index, &value) function pointer
    assign_function__ExtendedLaserScan__intrusion,  // assign(index, value) function pointer
    resize_function__ExtendedLaserScan__intrusion  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ExtendedLaserScan_message_members = {
  "sick_safetyscanners2_interfaces::msg",  // message namespace
  "ExtendedLaserScan",  // message name
  4,  // number of fields
  sizeof(sick_safetyscanners2_interfaces::msg::ExtendedLaserScan),
  ExtendedLaserScan_message_member_array,  // message members
  ExtendedLaserScan_init_function,  // function to initialize message memory (memory has to be allocated)
  ExtendedLaserScan_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ExtendedLaserScan_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ExtendedLaserScan_message_members,
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
get_message_type_support_handle<sick_safetyscanners2_interfaces::msg::ExtendedLaserScan>()
{
  return &::sick_safetyscanners2_interfaces::msg::rosidl_typesupport_introspection_cpp::ExtendedLaserScan_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, sick_safetyscanners2_interfaces, msg, ExtendedLaserScan)() {
  return &::sick_safetyscanners2_interfaces::msg::rosidl_typesupport_introspection_cpp::ExtendedLaserScan_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
