// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/DataHeader.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__DATA_HEADER__TRAITS_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__DATA_HEADER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sick_safetyscanners2_interfaces/msg/detail/data_header__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace sick_safetyscanners2_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const DataHeader & msg,
  std::ostream & out)
{
  out << "{";
  // member: version_version
  {
    out << "version_version: ";
    rosidl_generator_traits::value_to_yaml(msg.version_version, out);
    out << ", ";
  }

  // member: version_major_version
  {
    out << "version_major_version: ";
    rosidl_generator_traits::value_to_yaml(msg.version_major_version, out);
    out << ", ";
  }

  // member: version_minor_version
  {
    out << "version_minor_version: ";
    rosidl_generator_traits::value_to_yaml(msg.version_minor_version, out);
    out << ", ";
  }

  // member: version_release
  {
    out << "version_release: ";
    rosidl_generator_traits::value_to_yaml(msg.version_release, out);
    out << ", ";
  }

  // member: serial_number_of_device
  {
    out << "serial_number_of_device: ";
    rosidl_generator_traits::value_to_yaml(msg.serial_number_of_device, out);
    out << ", ";
  }

  // member: serial_number_of_channel_plug
  {
    out << "serial_number_of_channel_plug: ";
    rosidl_generator_traits::value_to_yaml(msg.serial_number_of_channel_plug, out);
    out << ", ";
  }

  // member: channel_number
  {
    out << "channel_number: ";
    rosidl_generator_traits::value_to_yaml(msg.channel_number, out);
    out << ", ";
  }

  // member: sequence_number
  {
    out << "sequence_number: ";
    rosidl_generator_traits::value_to_yaml(msg.sequence_number, out);
    out << ", ";
  }

  // member: scan_number
  {
    out << "scan_number: ";
    rosidl_generator_traits::value_to_yaml(msg.scan_number, out);
    out << ", ";
  }

  // member: timestamp_date
  {
    out << "timestamp_date: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp_date, out);
    out << ", ";
  }

  // member: timestamp_time
  {
    out << "timestamp_time: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp_time, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DataHeader & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: version_version
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "version_version: ";
    rosidl_generator_traits::value_to_yaml(msg.version_version, out);
    out << "\n";
  }

  // member: version_major_version
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "version_major_version: ";
    rosidl_generator_traits::value_to_yaml(msg.version_major_version, out);
    out << "\n";
  }

  // member: version_minor_version
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "version_minor_version: ";
    rosidl_generator_traits::value_to_yaml(msg.version_minor_version, out);
    out << "\n";
  }

  // member: version_release
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "version_release: ";
    rosidl_generator_traits::value_to_yaml(msg.version_release, out);
    out << "\n";
  }

  // member: serial_number_of_device
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "serial_number_of_device: ";
    rosidl_generator_traits::value_to_yaml(msg.serial_number_of_device, out);
    out << "\n";
  }

  // member: serial_number_of_channel_plug
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "serial_number_of_channel_plug: ";
    rosidl_generator_traits::value_to_yaml(msg.serial_number_of_channel_plug, out);
    out << "\n";
  }

  // member: channel_number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "channel_number: ";
    rosidl_generator_traits::value_to_yaml(msg.channel_number, out);
    out << "\n";
  }

  // member: sequence_number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sequence_number: ";
    rosidl_generator_traits::value_to_yaml(msg.sequence_number, out);
    out << "\n";
  }

  // member: scan_number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "scan_number: ";
    rosidl_generator_traits::value_to_yaml(msg.scan_number, out);
    out << "\n";
  }

  // member: timestamp_date
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp_date: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp_date, out);
    out << "\n";
  }

  // member: timestamp_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp_time: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp_time, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DataHeader & msg, bool use_flow_style = false)
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
  const sick_safetyscanners2_interfaces::msg::DataHeader & msg,
  std::ostream & out, size_t indentation = 0)
{
  sick_safetyscanners2_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sick_safetyscanners2_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const sick_safetyscanners2_interfaces::msg::DataHeader & msg)
{
  return sick_safetyscanners2_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sick_safetyscanners2_interfaces::msg::DataHeader>()
{
  return "sick_safetyscanners2_interfaces::msg::DataHeader";
}

template<>
inline const char * name<sick_safetyscanners2_interfaces::msg::DataHeader>()
{
  return "sick_safetyscanners2_interfaces/msg/DataHeader";
}

template<>
struct has_fixed_size<sick_safetyscanners2_interfaces::msg::DataHeader>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<sick_safetyscanners2_interfaces::msg::DataHeader>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<sick_safetyscanners2_interfaces::msg::DataHeader>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__DATA_HEADER__TRAITS_HPP_
