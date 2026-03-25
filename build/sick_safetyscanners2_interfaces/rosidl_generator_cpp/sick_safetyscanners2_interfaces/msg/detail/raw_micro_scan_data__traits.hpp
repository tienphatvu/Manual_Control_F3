// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/RawMicroScanData.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__RAW_MICRO_SCAN_DATA__TRAITS_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__RAW_MICRO_SCAN_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sick_safetyscanners2_interfaces/msg/detail/raw_micro_scan_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "sick_safetyscanners2_interfaces/msg/detail/data_header__traits.hpp"
// Member 'derived_values'
#include "sick_safetyscanners2_interfaces/msg/detail/derived_values__traits.hpp"
// Member 'general_system_state'
#include "sick_safetyscanners2_interfaces/msg/detail/general_system_state__traits.hpp"
// Member 'measurement_data'
#include "sick_safetyscanners2_interfaces/msg/detail/measurement_data__traits.hpp"
// Member 'intrusion_data'
#include "sick_safetyscanners2_interfaces/msg/detail/intrusion_data__traits.hpp"
// Member 'application_data'
#include "sick_safetyscanners2_interfaces/msg/detail/application_data__traits.hpp"

namespace sick_safetyscanners2_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const RawMicroScanData & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: derived_values
  {
    out << "derived_values: ";
    to_flow_style_yaml(msg.derived_values, out);
    out << ", ";
  }

  // member: general_system_state
  {
    out << "general_system_state: ";
    to_flow_style_yaml(msg.general_system_state, out);
    out << ", ";
  }

  // member: measurement_data
  {
    out << "measurement_data: ";
    to_flow_style_yaml(msg.measurement_data, out);
    out << ", ";
  }

  // member: intrusion_data
  {
    out << "intrusion_data: ";
    to_flow_style_yaml(msg.intrusion_data, out);
    out << ", ";
  }

  // member: application_data
  {
    out << "application_data: ";
    to_flow_style_yaml(msg.application_data, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RawMicroScanData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: derived_values
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "derived_values:\n";
    to_block_style_yaml(msg.derived_values, out, indentation + 2);
  }

  // member: general_system_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "general_system_state:\n";
    to_block_style_yaml(msg.general_system_state, out, indentation + 2);
  }

  // member: measurement_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "measurement_data:\n";
    to_block_style_yaml(msg.measurement_data, out, indentation + 2);
  }

  // member: intrusion_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "intrusion_data:\n";
    to_block_style_yaml(msg.intrusion_data, out, indentation + 2);
  }

  // member: application_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "application_data:\n";
    to_block_style_yaml(msg.application_data, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RawMicroScanData & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace sick_safetyscanners2_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use sick_safetyscanners2_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const sick_safetyscanners2_interfaces::msg::RawMicroScanData & msg,
  std::ostream & out, size_t indentation = 0)
{
  sick_safetyscanners2_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sick_safetyscanners2_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const sick_safetyscanners2_interfaces::msg::RawMicroScanData & msg)
{
  return sick_safetyscanners2_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sick_safetyscanners2_interfaces::msg::RawMicroScanData>()
{
  return "sick_safetyscanners2_interfaces::msg::RawMicroScanData";
}

template<>
inline const char * name<sick_safetyscanners2_interfaces::msg::RawMicroScanData>()
{
  return "sick_safetyscanners2_interfaces/msg/RawMicroScanData";
}

template<>
struct has_fixed_size<sick_safetyscanners2_interfaces::msg::RawMicroScanData>
  : std::integral_constant<bool, has_fixed_size<sick_safetyscanners2_interfaces::msg::ApplicationData>::value && has_fixed_size<sick_safetyscanners2_interfaces::msg::DataHeader>::value && has_fixed_size<sick_safetyscanners2_interfaces::msg::DerivedValues>::value && has_fixed_size<sick_safetyscanners2_interfaces::msg::GeneralSystemState>::value && has_fixed_size<sick_safetyscanners2_interfaces::msg::IntrusionData>::value && has_fixed_size<sick_safetyscanners2_interfaces::msg::MeasurementData>::value> {};

template<>
struct has_bounded_size<sick_safetyscanners2_interfaces::msg::RawMicroScanData>
  : std::integral_constant<bool, has_bounded_size<sick_safetyscanners2_interfaces::msg::ApplicationData>::value && has_bounded_size<sick_safetyscanners2_interfaces::msg::DataHeader>::value && has_bounded_size<sick_safetyscanners2_interfaces::msg::DerivedValues>::value && has_bounded_size<sick_safetyscanners2_interfaces::msg::GeneralSystemState>::value && has_bounded_size<sick_safetyscanners2_interfaces::msg::IntrusionData>::value && has_bounded_size<sick_safetyscanners2_interfaces::msg::MeasurementData>::value> {};

template<>
struct is_message<sick_safetyscanners2_interfaces::msg::RawMicroScanData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__RAW_MICRO_SCAN_DATA__TRAITS_HPP_
