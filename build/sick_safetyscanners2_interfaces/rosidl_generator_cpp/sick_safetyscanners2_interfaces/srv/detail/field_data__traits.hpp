// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sick_safetyscanners2_interfaces:srv/FieldData.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__SRV__DETAIL__FIELD_DATA__TRAITS_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__SRV__DETAIL__FIELD_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sick_safetyscanners2_interfaces/srv/detail/field_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace sick_safetyscanners2_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const FieldData_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FieldData_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FieldData_Request & msg, bool use_flow_style = false)
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
  const sick_safetyscanners2_interfaces::srv::FieldData_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  sick_safetyscanners2_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sick_safetyscanners2_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const sick_safetyscanners2_interfaces::srv::FieldData_Request & msg)
{
  return sick_safetyscanners2_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<sick_safetyscanners2_interfaces::srv::FieldData_Request>()
{
  return "sick_safetyscanners2_interfaces::srv::FieldData_Request";
}

template<>
inline const char * name<sick_safetyscanners2_interfaces::srv::FieldData_Request>()
{
  return "sick_safetyscanners2_interfaces/srv/FieldData_Request";
}

template<>
struct has_fixed_size<sick_safetyscanners2_interfaces::srv::FieldData_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<sick_safetyscanners2_interfaces::srv::FieldData_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<sick_safetyscanners2_interfaces::srv::FieldData_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'fields'
#include "sick_safetyscanners2_interfaces/msg/detail/field__traits.hpp"
// Member 'monitoring_cases'
#include "sick_safetyscanners2_interfaces/msg/detail/monitoring_case__traits.hpp"

namespace sick_safetyscanners2_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const FieldData_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: fields
  {
    if (msg.fields.size() == 0) {
      out << "fields: []";
    } else {
      out << "fields: [";
      size_t pending_items = msg.fields.size();
      for (auto item : msg.fields) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: device_name
  {
    out << "device_name: ";
    rosidl_generator_traits::value_to_yaml(msg.device_name, out);
    out << ", ";
  }

  // member: monitoring_cases
  {
    if (msg.monitoring_cases.size() == 0) {
      out << "monitoring_cases: []";
    } else {
      out << "monitoring_cases: [";
      size_t pending_items = msg.monitoring_cases.size();
      for (auto item : msg.monitoring_cases) {
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
  const FieldData_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
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
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: device_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "device_name: ";
    rosidl_generator_traits::value_to_yaml(msg.device_name, out);
    out << "\n";
  }

  // member: monitoring_cases
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.monitoring_cases.size() == 0) {
      out << "monitoring_cases: []\n";
    } else {
      out << "monitoring_cases:\n";
      for (auto item : msg.monitoring_cases) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FieldData_Response & msg, bool use_flow_style = false)
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
  const sick_safetyscanners2_interfaces::srv::FieldData_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  sick_safetyscanners2_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sick_safetyscanners2_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const sick_safetyscanners2_interfaces::srv::FieldData_Response & msg)
{
  return sick_safetyscanners2_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<sick_safetyscanners2_interfaces::srv::FieldData_Response>()
{
  return "sick_safetyscanners2_interfaces::srv::FieldData_Response";
}

template<>
inline const char * name<sick_safetyscanners2_interfaces::srv::FieldData_Response>()
{
  return "sick_safetyscanners2_interfaces/srv/FieldData_Response";
}

template<>
struct has_fixed_size<sick_safetyscanners2_interfaces::srv::FieldData_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<sick_safetyscanners2_interfaces::srv::FieldData_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<sick_safetyscanners2_interfaces::srv::FieldData_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<sick_safetyscanners2_interfaces::srv::FieldData>()
{
  return "sick_safetyscanners2_interfaces::srv::FieldData";
}

template<>
inline const char * name<sick_safetyscanners2_interfaces::srv::FieldData>()
{
  return "sick_safetyscanners2_interfaces/srv/FieldData";
}

template<>
struct has_fixed_size<sick_safetyscanners2_interfaces::srv::FieldData>
  : std::integral_constant<
    bool,
    has_fixed_size<sick_safetyscanners2_interfaces::srv::FieldData_Request>::value &&
    has_fixed_size<sick_safetyscanners2_interfaces::srv::FieldData_Response>::value
  >
{
};

template<>
struct has_bounded_size<sick_safetyscanners2_interfaces::srv::FieldData>
  : std::integral_constant<
    bool,
    has_bounded_size<sick_safetyscanners2_interfaces::srv::FieldData_Request>::value &&
    has_bounded_size<sick_safetyscanners2_interfaces::srv::FieldData_Response>::value
  >
{
};

template<>
struct is_service<sick_safetyscanners2_interfaces::srv::FieldData>
  : std::true_type
{
};

template<>
struct is_service_request<sick_safetyscanners2_interfaces::srv::FieldData_Request>
  : std::true_type
{
};

template<>
struct is_service_response<sick_safetyscanners2_interfaces::srv::FieldData_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__SRV__DETAIL__FIELD_DATA__TRAITS_HPP_
