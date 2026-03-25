// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sick_safetyscanners2_interfaces:srv/StatusOverview.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__SRV__DETAIL__STATUS_OVERVIEW__TRAITS_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__SRV__DETAIL__STATUS_OVERVIEW__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sick_safetyscanners2_interfaces/srv/detail/status_overview__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace sick_safetyscanners2_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const StatusOverview_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StatusOverview_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StatusOverview_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace sick_safetyscanners2_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use sick_safetyscanners2_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const sick_safetyscanners2_interfaces::srv::StatusOverview_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  sick_safetyscanners2_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sick_safetyscanners2_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const sick_safetyscanners2_interfaces::srv::StatusOverview_Request & msg)
{
  return sick_safetyscanners2_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<sick_safetyscanners2_interfaces::srv::StatusOverview_Request>()
{
  return "sick_safetyscanners2_interfaces::srv::StatusOverview_Request";
}

template<>
inline const char * name<sick_safetyscanners2_interfaces::srv::StatusOverview_Request>()
{
  return "sick_safetyscanners2_interfaces/srv/StatusOverview_Request";
}

template<>
struct has_fixed_size<sick_safetyscanners2_interfaces::srv::StatusOverview_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<sick_safetyscanners2_interfaces::srv::StatusOverview_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<sick_safetyscanners2_interfaces::srv::StatusOverview_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace sick_safetyscanners2_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const StatusOverview_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: version_c_version
  {
    out << "version_c_version: ";
    rosidl_generator_traits::value_to_yaml(msg.version_c_version, out);
    out << ", ";
  }

  // member: version_major_version_number
  {
    out << "version_major_version_number: ";
    rosidl_generator_traits::value_to_yaml(msg.version_major_version_number, out);
    out << ", ";
  }

  // member: version_minor_version_number
  {
    out << "version_minor_version_number: ";
    rosidl_generator_traits::value_to_yaml(msg.version_minor_version_number, out);
    out << ", ";
  }

  // member: version_release_number
  {
    out << "version_release_number: ";
    rosidl_generator_traits::value_to_yaml(msg.version_release_number, out);
    out << ", ";
  }

  // member: device_state
  {
    out << "device_state: ";
    rosidl_generator_traits::value_to_yaml(msg.device_state, out);
    out << ", ";
  }

  // member: config_state
  {
    out << "config_state: ";
    rosidl_generator_traits::value_to_yaml(msg.config_state, out);
    out << ", ";
  }

  // member: application_state
  {
    out << "application_state: ";
    rosidl_generator_traits::value_to_yaml(msg.application_state, out);
    out << ", ";
  }

  // member: current_time_power_on_count
  {
    out << "current_time_power_on_count: ";
    rosidl_generator_traits::value_to_yaml(msg.current_time_power_on_count, out);
    out << ", ";
  }

  // member: current_time
  {
    out << "current_time: ";
    rosidl_generator_traits::value_to_yaml(msg.current_time, out);
    out << ", ";
  }

  // member: current_time_time
  {
    out << "current_time_time: ";
    rosidl_generator_traits::value_to_yaml(msg.current_time_time, out);
    out << ", ";
  }

  // member: current_time_date
  {
    out << "current_time_date: ";
    rosidl_generator_traits::value_to_yaml(msg.current_time_date, out);
    out << ", ";
  }

  // member: error_info_code
  {
    out << "error_info_code: ";
    rosidl_generator_traits::value_to_yaml(msg.error_info_code, out);
    out << ", ";
  }

  // member: error_info_time
  {
    out << "error_info_time: ";
    rosidl_generator_traits::value_to_yaml(msg.error_info_time, out);
    out << ", ";
  }

  // member: error_info_time_time
  {
    out << "error_info_time_time: ";
    rosidl_generator_traits::value_to_yaml(msg.error_info_time_time, out);
    out << ", ";
  }

  // member: error_info_time_date
  {
    out << "error_info_time_date: ";
    rosidl_generator_traits::value_to_yaml(msg.error_info_time_date, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StatusOverview_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: version_c_version
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "version_c_version: ";
    rosidl_generator_traits::value_to_yaml(msg.version_c_version, out);
    out << "\n";
  }

  // member: version_major_version_number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "version_major_version_number: ";
    rosidl_generator_traits::value_to_yaml(msg.version_major_version_number, out);
    out << "\n";
  }

  // member: version_minor_version_number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "version_minor_version_number: ";
    rosidl_generator_traits::value_to_yaml(msg.version_minor_version_number, out);
    out << "\n";
  }

  // member: version_release_number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "version_release_number: ";
    rosidl_generator_traits::value_to_yaml(msg.version_release_number, out);
    out << "\n";
  }

  // member: device_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "device_state: ";
    rosidl_generator_traits::value_to_yaml(msg.device_state, out);
    out << "\n";
  }

  // member: config_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "config_state: ";
    rosidl_generator_traits::value_to_yaml(msg.config_state, out);
    out << "\n";
  }

  // member: application_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "application_state: ";
    rosidl_generator_traits::value_to_yaml(msg.application_state, out);
    out << "\n";
  }

  // member: current_time_power_on_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_time_power_on_count: ";
    rosidl_generator_traits::value_to_yaml(msg.current_time_power_on_count, out);
    out << "\n";
  }

  // member: current_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_time: ";
    rosidl_generator_traits::value_to_yaml(msg.current_time, out);
    out << "\n";
  }

  // member: current_time_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_time_time: ";
    rosidl_generator_traits::value_to_yaml(msg.current_time_time, out);
    out << "\n";
  }

  // member: current_time_date
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_time_date: ";
    rosidl_generator_traits::value_to_yaml(msg.current_time_date, out);
    out << "\n";
  }

  // member: error_info_code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_info_code: ";
    rosidl_generator_traits::value_to_yaml(msg.error_info_code, out);
    out << "\n";
  }

  // member: error_info_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_info_time: ";
    rosidl_generator_traits::value_to_yaml(msg.error_info_time, out);
    out << "\n";
  }

  // member: error_info_time_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_info_time_time: ";
    rosidl_generator_traits::value_to_yaml(msg.error_info_time_time, out);
    out << "\n";
  }

  // member: error_info_time_date
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_info_time_date: ";
    rosidl_generator_traits::value_to_yaml(msg.error_info_time_date, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StatusOverview_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace sick_safetyscanners2_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use sick_safetyscanners2_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const sick_safetyscanners2_interfaces::srv::StatusOverview_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  sick_safetyscanners2_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sick_safetyscanners2_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const sick_safetyscanners2_interfaces::srv::StatusOverview_Response & msg)
{
  return sick_safetyscanners2_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<sick_safetyscanners2_interfaces::srv::StatusOverview_Response>()
{
  return "sick_safetyscanners2_interfaces::srv::StatusOverview_Response";
}

template<>
inline const char * name<sick_safetyscanners2_interfaces::srv::StatusOverview_Response>()
{
  return "sick_safetyscanners2_interfaces/srv/StatusOverview_Response";
}

template<>
struct has_fixed_size<sick_safetyscanners2_interfaces::srv::StatusOverview_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<sick_safetyscanners2_interfaces::srv::StatusOverview_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<sick_safetyscanners2_interfaces::srv::StatusOverview_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<sick_safetyscanners2_interfaces::srv::StatusOverview>()
{
  return "sick_safetyscanners2_interfaces::srv::StatusOverview";
}

template<>
inline const char * name<sick_safetyscanners2_interfaces::srv::StatusOverview>()
{
  return "sick_safetyscanners2_interfaces/srv/StatusOverview";
}

template<>
struct has_fixed_size<sick_safetyscanners2_interfaces::srv::StatusOverview>
  : std::integral_constant<
    bool,
    has_fixed_size<sick_safetyscanners2_interfaces::srv::StatusOverview_Request>::value &&
    has_fixed_size<sick_safetyscanners2_interfaces::srv::StatusOverview_Response>::value
  >
{
};

template<>
struct has_bounded_size<sick_safetyscanners2_interfaces::srv::StatusOverview>
  : std::integral_constant<
    bool,
    has_bounded_size<sick_safetyscanners2_interfaces::srv::StatusOverview_Request>::value &&
    has_bounded_size<sick_safetyscanners2_interfaces::srv::StatusOverview_Response>::value
  >
{
};

template<>
struct is_service<sick_safetyscanners2_interfaces::srv::StatusOverview>
  : std::true_type
{
};

template<>
struct is_service_request<sick_safetyscanners2_interfaces::srv::StatusOverview_Request>
  : std::true_type
{
};

template<>
struct is_service_response<sick_safetyscanners2_interfaces::srv::StatusOverview_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__SRV__DETAIL__STATUS_OVERVIEW__TRAITS_HPP_
