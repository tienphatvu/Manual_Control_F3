// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/DerivedValues.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__DERIVED_VALUES__TRAITS_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__DERIVED_VALUES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sick_safetyscanners2_interfaces/msg/detail/derived_values__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace sick_safetyscanners2_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const DerivedValues & msg,
  std::ostream & out)
{
  out << "{";
  // member: multiplication_factor
  {
    out << "multiplication_factor: ";
    rosidl_generator_traits::value_to_yaml(msg.multiplication_factor, out);
    out << ", ";
  }

  // member: number_of_beams
  {
    out << "number_of_beams: ";
    rosidl_generator_traits::value_to_yaml(msg.number_of_beams, out);
    out << ", ";
  }

  // member: scan_time
  {
    out << "scan_time: ";
    rosidl_generator_traits::value_to_yaml(msg.scan_time, out);
    out << ", ";
  }

  // member: start_angle
  {
    out << "start_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.start_angle, out);
    out << ", ";
  }

  // member: angular_beam_resolution
  {
    out << "angular_beam_resolution: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_beam_resolution, out);
    out << ", ";
  }

  // member: interbeam_period
  {
    out << "interbeam_period: ";
    rosidl_generator_traits::value_to_yaml(msg.interbeam_period, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DerivedValues & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: multiplication_factor
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "multiplication_factor: ";
    rosidl_generator_traits::value_to_yaml(msg.multiplication_factor, out);
    out << "\n";
  }

  // member: number_of_beams
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "number_of_beams: ";
    rosidl_generator_traits::value_to_yaml(msg.number_of_beams, out);
    out << "\n";
  }

  // member: scan_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "scan_time: ";
    rosidl_generator_traits::value_to_yaml(msg.scan_time, out);
    out << "\n";
  }

  // member: start_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.start_angle, out);
    out << "\n";
  }

  // member: angular_beam_resolution
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angular_beam_resolution: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_beam_resolution, out);
    out << "\n";
  }

  // member: interbeam_period
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "interbeam_period: ";
    rosidl_generator_traits::value_to_yaml(msg.interbeam_period, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DerivedValues & msg, bool use_flow_style = false)
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
  const sick_safetyscanners2_interfaces::msg::DerivedValues & msg,
  std::ostream & out, size_t indentation = 0)
{
  sick_safetyscanners2_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sick_safetyscanners2_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const sick_safetyscanners2_interfaces::msg::DerivedValues & msg)
{
  return sick_safetyscanners2_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sick_safetyscanners2_interfaces::msg::DerivedValues>()
{
  return "sick_safetyscanners2_interfaces::msg::DerivedValues";
}

template<>
inline const char * name<sick_safetyscanners2_interfaces::msg::DerivedValues>()
{
  return "sick_safetyscanners2_interfaces/msg/DerivedValues";
}

template<>
struct has_fixed_size<sick_safetyscanners2_interfaces::msg::DerivedValues>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<sick_safetyscanners2_interfaces::msg::DerivedValues>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<sick_safetyscanners2_interfaces::msg::DerivedValues>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__DERIVED_VALUES__TRAITS_HPP_
