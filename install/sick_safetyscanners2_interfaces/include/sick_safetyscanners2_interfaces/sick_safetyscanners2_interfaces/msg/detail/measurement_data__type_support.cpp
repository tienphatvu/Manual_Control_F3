// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from sick_safetyscanners2_interfaces:msg/MeasurementData.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "sick_safetyscanners2_interfaces/msg/detail/measurement_data__struct.hpp"
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

void MeasurementData_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) sick_safetyscanners2_interfaces::msg::MeasurementData(_init);
}

void MeasurementData_fini_function(void * message_memory)
{
  auto typed_message = static_cast<sick_safetyscanners2_interfaces::msg::MeasurementData *>(message_memory);
  typed_message->~MeasurementData();
}

size_t size_function__MeasurementData__scan_points(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<sick_safetyscanners2_interfaces::msg::ScanPoint> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MeasurementData__scan_points(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<sick_safetyscanners2_interfaces::msg::ScanPoint> *>(untyped_member);
  return &member[index];
}

void * get_function__MeasurementData__scan_points(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<sick_safetyscanners2_interfaces::msg::ScanPoint> *>(untyped_member);
  return &member[index];
}

void fetch_function__MeasurementData__scan_points(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const sick_safetyscanners2_interfaces::msg::ScanPoint *>(
    get_const_function__MeasurementData__scan_points(untyped_member, index));
  auto & value = *reinterpret_cast<sick_safetyscanners2_interfaces::msg::ScanPoint *>(untyped_value);
  value = item;
}

void assign_function__MeasurementData__scan_points(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<sick_safetyscanners2_interfaces::msg::ScanPoint *>(
    get_function__MeasurementData__scan_points(untyped_member, index));
  const auto & value = *reinterpret_cast<const sick_safetyscanners2_interfaces::msg::ScanPoint *>(untyped_value);
  item = value;
}

void resize_function__MeasurementData__scan_points(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<sick_safetyscanners2_interfaces::msg::ScanPoint> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MeasurementData_message_member_array[2] = {
  {
    "number_of_beams",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces::msg::MeasurementData, number_of_beams),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "scan_points",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<sick_safetyscanners2_interfaces::msg::ScanPoint>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces::msg::MeasurementData, scan_points),  // bytes offset in struct
    nullptr,  // default value
    size_function__MeasurementData__scan_points,  // size() function pointer
    get_const_function__MeasurementData__scan_points,  // get_const(index) function pointer
    get_function__MeasurementData__scan_points,  // get(index) function pointer
    fetch_function__MeasurementData__scan_points,  // fetch(index, &value) function pointer
    assign_function__MeasurementData__scan_points,  // assign(index, value) function pointer
    resize_function__MeasurementData__scan_points  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MeasurementData_message_members = {
  "sick_safetyscanners2_interfaces::msg",  // message namespace
  "MeasurementData",  // message name
  2,  // number of fields
  sizeof(sick_safetyscanners2_interfaces::msg::MeasurementData),
  MeasurementData_message_member_array,  // message members
  MeasurementData_init_function,  // function to initialize message memory (memory has to be allocated)
  MeasurementData_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MeasurementData_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MeasurementData_message_members,
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
get_message_type_support_handle<sick_safetyscanners2_interfaces::msg::MeasurementData>()
{
  return &::sick_safetyscanners2_interfaces::msg::rosidl_typesupport_introspection_cpp::MeasurementData_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, sick_safetyscanners2_interfaces, msg, MeasurementData)() {
  return &::sick_safetyscanners2_interfaces::msg::rosidl_typesupport_introspection_cpp::MeasurementData_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
