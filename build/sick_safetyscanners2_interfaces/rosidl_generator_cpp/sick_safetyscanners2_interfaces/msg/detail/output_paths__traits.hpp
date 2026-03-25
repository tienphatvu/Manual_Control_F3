// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/OutputPaths.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__OUTPUT_PATHS__TRAITS_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__OUTPUT_PATHS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sick_safetyscanners2_interfaces/msg/detail/output_paths__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace sick_safetyscanners2_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const OutputPaths & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    if (msg.status.size() == 0) {
      out << "status: []";
    } else {
      out << "status: [";
      size_t pending_items = msg.status.size();
      for (auto item : msg.status) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: is_safe
  {
    if (msg.is_safe.size() == 0) {
      out << "is_safe: []";
    } else {
      out << "is_safe: [";
      size_t pending_items = msg.is_safe.size();
      for (auto item : msg.is_safe) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: is_valid
  {
    if (msg.is_valid.size() == 0) {
      out << "is_valid: []";
    } else {
      out << "is_valid: [";
      size_t pending_items = msg.is_valid.size();
      for (auto item : msg.is_valid) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: active_monitoring_case
  {
    out << "active_monitoring_case: ";
    rosidl_generator_traits::value_to_yaml(msg.active_monitoring_case, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OutputPaths & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.status.size() == 0) {
      out << "status: []\n";
    } else {
      out << "status:\n";
      for (auto item : msg.status) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: is_safe
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.is_safe.size() == 0) {
      out << "is_safe: []\n";
    } else {
      out << "is_safe:\n";
      for (auto item : msg.is_safe) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: is_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.is_valid.size() == 0) {
      out << "is_valid: []\n";
    } else {
      out << "is_valid:\n";
      for (auto item : msg.is_valid) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: active_monitoring_case
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "active_monitoring_case: ";
    rosidl_generator_traits::value_to_yaml(msg.active_monitoring_case, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OutputPaths & msg, bool use_flow_style = false)
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
  const sick_safetyscanners2_interfaces::msg::OutputPaths & msg,
  std::ostream & out, size_t indentation = 0)
{
  sick_safetyscanners2_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sick_safetyscanners2_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const sick_safetyscanners2_interfaces::msg::OutputPaths & msg)
{
  return sick_safetyscanners2_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sick_safetyscanners2_interfaces::msg::OutputPaths>()
{
  return "sick_safetyscanners2_interfaces::msg::OutputPaths";
}

template<>
inline const char * name<sick_safetyscanners2_interfaces::msg::OutputPaths>()
{
  return "sick_safetyscanners2_interfaces/msg/OutputPaths";
}

template<>
struct has_fixed_size<sick_safetyscanners2_interfaces::msg::OutputPaths>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<sick_safetyscanners2_interfaces::msg::OutputPaths>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<sick_safetyscanners2_interfaces::msg::OutputPaths>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__OUTPUT_PATHS__TRAITS_HPP_
