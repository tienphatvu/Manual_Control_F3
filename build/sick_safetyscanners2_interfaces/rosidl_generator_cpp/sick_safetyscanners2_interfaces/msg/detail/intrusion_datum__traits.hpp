// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/IntrusionDatum.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__INTRUSION_DATUM__TRAITS_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__INTRUSION_DATUM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sick_safetyscanners2_interfaces/msg/detail/intrusion_datum__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace sick_safetyscanners2_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const IntrusionDatum & msg,
  std::ostream & out)
{
  out << "{";
  // member: size
  {
    out << "size: ";
    rosidl_generator_traits::value_to_yaml(msg.size, out);
    out << ", ";
  }

  // member: flags
  {
    if (msg.flags.size() == 0) {
      out << "flags: []";
    } else {
      out << "flags: [";
      size_t pending_items = msg.flags.size();
      for (auto item : msg.flags) {
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
  const IntrusionDatum & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: size
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "size: ";
    rosidl_generator_traits::value_to_yaml(msg.size, out);
    out << "\n";
  }

  // member: flags
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.flags.size() == 0) {
      out << "flags: []\n";
    } else {
      out << "flags:\n";
      for (auto item : msg.flags) {
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

inline std::string to_yaml(const IntrusionDatum & msg, bool use_flow_style = false)
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
  const sick_safetyscanners2_interfaces::msg::IntrusionDatum & msg,
  std::ostream & out, size_t indentation = 0)
{
  sick_safetyscanners2_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sick_safetyscanners2_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const sick_safetyscanners2_interfaces::msg::IntrusionDatum & msg)
{
  return sick_safetyscanners2_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sick_safetyscanners2_interfaces::msg::IntrusionDatum>()
{
  return "sick_safetyscanners2_interfaces::msg::IntrusionDatum";
}

template<>
inline const char * name<sick_safetyscanners2_interfaces::msg::IntrusionDatum>()
{
  return "sick_safetyscanners2_interfaces/msg/IntrusionDatum";
}

template<>
struct has_fixed_size<sick_safetyscanners2_interfaces::msg::IntrusionDatum>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<sick_safetyscanners2_interfaces::msg::IntrusionDatum>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<sick_safetyscanners2_interfaces::msg::IntrusionDatum>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__INTRUSION_DATUM__TRAITS_HPP_
