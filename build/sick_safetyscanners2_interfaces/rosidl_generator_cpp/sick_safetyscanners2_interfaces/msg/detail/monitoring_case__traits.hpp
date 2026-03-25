// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/MonitoringCase.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__MONITORING_CASE__TRAITS_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__MONITORING_CASE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sick_safetyscanners2_interfaces/msg/detail/monitoring_case__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace sick_safetyscanners2_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const MonitoringCase & msg,
  std::ostream & out)
{
  out << "{";
  // member: monitoring_case_number
  {
    out << "monitoring_case_number: ";
    rosidl_generator_traits::value_to_yaml(msg.monitoring_case_number, out);
    out << ", ";
  }

  // member: fields
  {
    if (msg.fields.size() == 0) {
      out << "fields: []";
    } else {
      out << "fields: [";
      size_t pending_items = msg.fields.size();
      for (auto item : msg.fields) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: fields_valid
  {
    if (msg.fields_valid.size() == 0) {
      out << "fields_valid: []";
    } else {
      out << "fields_valid: [";
      size_t pending_items = msg.fields_valid.size();
      for (auto item : msg.fields_valid) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MonitoringCase & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: monitoring_case_number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "monitoring_case_number: ";
    rosidl_generator_traits::value_to_yaml(msg.monitoring_case_number, out);
    out << "\n";
  }

  // member: fields
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.fields.size() == 0) {
      out << "fields: []\n";
    } else {
      out << "fields:\n";
      for (auto item : msg.fields) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: fields_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.fields_valid.size() == 0) {
      out << "fields_valid: []\n";
    } else {
      out << "fields_valid:\n";
      for (auto item : msg.fields_valid) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MonitoringCase & msg, bool use_flow_style = false)
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
  const sick_safetyscanners2_interfaces::msg::MonitoringCase & msg,
  std::ostream & out, size_t indentation = 0)
{
  sick_safetyscanners2_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sick_safetyscanners2_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const sick_safetyscanners2_interfaces::msg::MonitoringCase & msg)
{
  return sick_safetyscanners2_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sick_safetyscanners2_interfaces::msg::MonitoringCase>()
{
  return "sick_safetyscanners2_interfaces::msg::MonitoringCase";
}

template<>
inline const char * name<sick_safetyscanners2_interfaces::msg::MonitoringCase>()
{
  return "sick_safetyscanners2_interfaces/msg/MonitoringCase";
}

template<>
struct has_fixed_size<sick_safetyscanners2_interfaces::msg::MonitoringCase>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<sick_safetyscanners2_interfaces::msg::MonitoringCase>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<sick_safetyscanners2_interfaces::msg::MonitoringCase>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__MONITORING_CASE__TRAITS_HPP_
