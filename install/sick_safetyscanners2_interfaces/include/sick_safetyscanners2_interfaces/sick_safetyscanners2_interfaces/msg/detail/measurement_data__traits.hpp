// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/MeasurementData.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__MEASUREMENT_DATA__TRAITS_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__MEASUREMENT_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sick_safetyscanners2_interfaces/msg/detail/measurement_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'scan_points'
#include "sick_safetyscanners2_interfaces/msg/detail/scan_point__traits.hpp"

namespace sick_safetyscanners2_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const MeasurementData & msg,
  std::ostream & out)
{
  out << "{";
  // member: number_of_beams
  {
    out << "number_of_beams: ";
    rosidl_generator_traits::value_to_yaml(msg.number_of_beams, out);
    out << ", ";
  }

  // member: scan_points
  {
    if (msg.scan_points.size() == 0) {
      out << "scan_points: []";
    } else {
      out << "scan_points: [";
      size_t pending_items = msg.scan_points.size();
      for (auto item : msg.scan_points) {
        to_flow_style_yaml(item, out);
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
  const MeasurementData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: number_of_beams
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "number_of_beams: ";
    rosidl_generator_traits::value_to_yaml(msg.number_of_beams, out);
    out << "\n";
  }

  // member: scan_points
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.scan_points.size() == 0) {
      out << "scan_points: []\n";
    } else {
      out << "scan_points:\n";
      for (auto item : msg.scan_points) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MeasurementData & msg, bool use_flow_style = false)
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
  const sick_safetyscanners2_interfaces::msg::MeasurementData & msg,
  std::ostream & out, size_t indentation = 0)
{
  sick_safetyscanners2_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sick_safetyscanners2_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const sick_safetyscanners2_interfaces::msg::MeasurementData & msg)
{
  return sick_safetyscanners2_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sick_safetyscanners2_interfaces::msg::MeasurementData>()
{
  return "sick_safetyscanners2_interfaces::msg::MeasurementData";
}

template<>
inline const char * name<sick_safetyscanners2_interfaces::msg::MeasurementData>()
{
  return "sick_safetyscanners2_interfaces/msg/MeasurementData";
}

template<>
struct has_fixed_size<sick_safetyscanners2_interfaces::msg::MeasurementData>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<sick_safetyscanners2_interfaces::msg::MeasurementData>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<sick_safetyscanners2_interfaces::msg::MeasurementData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__MEASUREMENT_DATA__TRAITS_HPP_
