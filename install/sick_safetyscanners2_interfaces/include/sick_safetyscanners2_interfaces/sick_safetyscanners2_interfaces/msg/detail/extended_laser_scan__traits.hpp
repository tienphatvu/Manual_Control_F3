// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/ExtendedLaserScan.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__EXTENDED_LASER_SCAN__TRAITS_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__EXTENDED_LASER_SCAN__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sick_safetyscanners2_interfaces/msg/detail/extended_laser_scan__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'laser_scan'
#include "sensor_msgs/msg/detail/laser_scan__traits.hpp"

namespace sick_safetyscanners2_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ExtendedLaserScan & msg,
  std::ostream & out)
{
  out << "{";
  // member: laser_scan
  {
    out << "laser_scan: ";
    to_flow_style_yaml(msg.laser_scan, out);
    out << ", ";
  }

  // member: reflektor_status
  {
    if (msg.reflektor_status.size() == 0) {
      out << "reflektor_status: []";
    } else {
      out << "reflektor_status: [";
      size_t pending_items = msg.reflektor_status.size();
      for (auto item : msg.reflektor_status) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: reflektor_median
  {
    if (msg.reflektor_median.size() == 0) {
      out << "reflektor_median: []";
    } else {
      out << "reflektor_median: [";
      size_t pending_items = msg.reflektor_median.size();
      for (auto item : msg.reflektor_median) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: intrusion
  {
    if (msg.intrusion.size() == 0) {
      out << "intrusion: []";
    } else {
      out << "intrusion: [";
      size_t pending_items = msg.intrusion.size();
      for (auto item : msg.intrusion) {
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
  const ExtendedLaserScan & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: laser_scan
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "laser_scan:\n";
    to_block_style_yaml(msg.laser_scan, out, indentation + 2);
  }

  // member: reflektor_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.reflektor_status.size() == 0) {
      out << "reflektor_status: []\n";
    } else {
      out << "reflektor_status:\n";
      for (auto item : msg.reflektor_status) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: reflektor_median
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.reflektor_median.size() == 0) {
      out << "reflektor_median: []\n";
    } else {
      out << "reflektor_median:\n";
      for (auto item : msg.reflektor_median) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: intrusion
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.intrusion.size() == 0) {
      out << "intrusion: []\n";
    } else {
      out << "intrusion:\n";
      for (auto item : msg.intrusion) {
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

inline std::string to_yaml(const ExtendedLaserScan & msg, bool use_flow_style = false)
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
  const sick_safetyscanners2_interfaces::msg::ExtendedLaserScan & msg,
  std::ostream & out, size_t indentation = 0)
{
  sick_safetyscanners2_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sick_safetyscanners2_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const sick_safetyscanners2_interfaces::msg::ExtendedLaserScan & msg)
{
  return sick_safetyscanners2_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sick_safetyscanners2_interfaces::msg::ExtendedLaserScan>()
{
  return "sick_safetyscanners2_interfaces::msg::ExtendedLaserScan";
}

template<>
inline const char * name<sick_safetyscanners2_interfaces::msg::ExtendedLaserScan>()
{
  return "sick_safetyscanners2_interfaces/msg/ExtendedLaserScan";
}

template<>
struct has_fixed_size<sick_safetyscanners2_interfaces::msg::ExtendedLaserScan>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<sick_safetyscanners2_interfaces::msg::ExtendedLaserScan>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<sick_safetyscanners2_interfaces::msg::ExtendedLaserScan>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__EXTENDED_LASER_SCAN__TRAITS_HPP_
