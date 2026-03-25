// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/ScanPoint.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__SCAN_POINT__TRAITS_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__SCAN_POINT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sick_safetyscanners2_interfaces/msg/detail/scan_point__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace sick_safetyscanners2_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ScanPoint & msg,
  std::ostream & out)
{
  out << "{";
  // member: angle
  {
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << ", ";
  }

  // member: distance
  {
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << ", ";
  }

  // member: reflectivity
  {
    out << "reflectivity: ";
    rosidl_generator_traits::value_to_yaml(msg.reflectivity, out);
    out << ", ";
  }

  // member: valid
  {
    out << "valid: ";
    rosidl_generator_traits::value_to_yaml(msg.valid, out);
    out << ", ";
  }

  // member: infinite
  {
    out << "infinite: ";
    rosidl_generator_traits::value_to_yaml(msg.infinite, out);
    out << ", ";
  }

  // member: glare
  {
    out << "glare: ";
    rosidl_generator_traits::value_to_yaml(msg.glare, out);
    out << ", ";
  }

  // member: reflector
  {
    out << "reflector: ";
    rosidl_generator_traits::value_to_yaml(msg.reflector, out);
    out << ", ";
  }

  // member: contamination
  {
    out << "contamination: ";
    rosidl_generator_traits::value_to_yaml(msg.contamination, out);
    out << ", ";
  }

  // member: contamination_warning
  {
    out << "contamination_warning: ";
    rosidl_generator_traits::value_to_yaml(msg.contamination_warning, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ScanPoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << "\n";
  }

  // member: distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << "\n";
  }

  // member: reflectivity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reflectivity: ";
    rosidl_generator_traits::value_to_yaml(msg.reflectivity, out);
    out << "\n";
  }

  // member: valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "valid: ";
    rosidl_generator_traits::value_to_yaml(msg.valid, out);
    out << "\n";
  }

  // member: infinite
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "infinite: ";
    rosidl_generator_traits::value_to_yaml(msg.infinite, out);
    out << "\n";
  }

  // member: glare
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "glare: ";
    rosidl_generator_traits::value_to_yaml(msg.glare, out);
    out << "\n";
  }

  // member: reflector
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reflector: ";
    rosidl_generator_traits::value_to_yaml(msg.reflector, out);
    out << "\n";
  }

  // member: contamination
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "contamination: ";
    rosidl_generator_traits::value_to_yaml(msg.contamination, out);
    out << "\n";
  }

  // member: contamination_warning
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "contamination_warning: ";
    rosidl_generator_traits::value_to_yaml(msg.contamination_warning, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ScanPoint & msg, bool use_flow_style = false)
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
  const sick_safetyscanners2_interfaces::msg::ScanPoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  sick_safetyscanners2_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sick_safetyscanners2_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const sick_safetyscanners2_interfaces::msg::ScanPoint & msg)
{
  return sick_safetyscanners2_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sick_safetyscanners2_interfaces::msg::ScanPoint>()
{
  return "sick_safetyscanners2_interfaces::msg::ScanPoint";
}

template<>
inline const char * name<sick_safetyscanners2_interfaces::msg::ScanPoint>()
{
  return "sick_safetyscanners2_interfaces/msg/ScanPoint";
}

template<>
struct has_fixed_size<sick_safetyscanners2_interfaces::msg::ScanPoint>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<sick_safetyscanners2_interfaces::msg::ScanPoint>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<sick_safetyscanners2_interfaces::msg::ScanPoint>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__SCAN_POINT__TRAITS_HPP_
