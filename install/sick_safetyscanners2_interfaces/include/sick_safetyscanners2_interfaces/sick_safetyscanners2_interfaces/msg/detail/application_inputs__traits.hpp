// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/ApplicationInputs.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_INPUTS__TRAITS_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_INPUTS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sick_safetyscanners2_interfaces/msg/detail/application_inputs__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace sick_safetyscanners2_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ApplicationInputs & msg,
  std::ostream & out)
{
  out << "{";
  // member: unsafe_inputs_input_sources
  {
    if (msg.unsafe_inputs_input_sources.size() == 0) {
      out << "unsafe_inputs_input_sources: []";
    } else {
      out << "unsafe_inputs_input_sources: [";
      size_t pending_items = msg.unsafe_inputs_input_sources.size();
      for (auto item : msg.unsafe_inputs_input_sources) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: unsafe_inputs_flags
  {
    if (msg.unsafe_inputs_flags.size() == 0) {
      out << "unsafe_inputs_flags: []";
    } else {
      out << "unsafe_inputs_flags: [";
      size_t pending_items = msg.unsafe_inputs_flags.size();
      for (auto item : msg.unsafe_inputs_flags) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: monitoring_case_number_inputs
  {
    if (msg.monitoring_case_number_inputs.size() == 0) {
      out << "monitoring_case_number_inputs: []";
    } else {
      out << "monitoring_case_number_inputs: [";
      size_t pending_items = msg.monitoring_case_number_inputs.size();
      for (auto item : msg.monitoring_case_number_inputs) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: monitoring_case_number_inputs_flags
  {
    if (msg.monitoring_case_number_inputs_flags.size() == 0) {
      out << "monitoring_case_number_inputs_flags: []";
    } else {
      out << "monitoring_case_number_inputs_flags: [";
      size_t pending_items = msg.monitoring_case_number_inputs_flags.size();
      for (auto item : msg.monitoring_case_number_inputs_flags) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: linear_velocity_inputs_velocity_0
  {
    out << "linear_velocity_inputs_velocity_0: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity_inputs_velocity_0, out);
    out << ", ";
  }

  // member: linear_velocity_inputs_velocity_0_valid
  {
    out << "linear_velocity_inputs_velocity_0_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity_inputs_velocity_0_valid, out);
    out << ", ";
  }

  // member: linear_velocity_inputs_velocity_0_transmitted_safely
  {
    out << "linear_velocity_inputs_velocity_0_transmitted_safely: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity_inputs_velocity_0_transmitted_safely, out);
    out << ", ";
  }

  // member: linear_velocity_inputs_velocity_1
  {
    out << "linear_velocity_inputs_velocity_1: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity_inputs_velocity_1, out);
    out << ", ";
  }

  // member: linear_velocity_inputs_velocity_1_valid
  {
    out << "linear_velocity_inputs_velocity_1_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity_inputs_velocity_1_valid, out);
    out << ", ";
  }

  // member: linear_velocity_inputs_velocity_1_transmitted_safely
  {
    out << "linear_velocity_inputs_velocity_1_transmitted_safely: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity_inputs_velocity_1_transmitted_safely, out);
    out << ", ";
  }

  // member: sleep_mode_input
  {
    out << "sleep_mode_input: ";
    rosidl_generator_traits::value_to_yaml(msg.sleep_mode_input, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ApplicationInputs & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: unsafe_inputs_input_sources
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.unsafe_inputs_input_sources.size() == 0) {
      out << "unsafe_inputs_input_sources: []\n";
    } else {
      out << "unsafe_inputs_input_sources:\n";
      for (auto item : msg.unsafe_inputs_input_sources) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: unsafe_inputs_flags
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.unsafe_inputs_flags.size() == 0) {
      out << "unsafe_inputs_flags: []\n";
    } else {
      out << "unsafe_inputs_flags:\n";
      for (auto item : msg.unsafe_inputs_flags) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: monitoring_case_number_inputs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.monitoring_case_number_inputs.size() == 0) {
      out << "monitoring_case_number_inputs: []\n";
    } else {
      out << "monitoring_case_number_inputs:\n";
      for (auto item : msg.monitoring_case_number_inputs) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: monitoring_case_number_inputs_flags
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.monitoring_case_number_inputs_flags.size() == 0) {
      out << "monitoring_case_number_inputs_flags: []\n";
    } else {
      out << "monitoring_case_number_inputs_flags:\n";
      for (auto item : msg.monitoring_case_number_inputs_flags) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: linear_velocity_inputs_velocity_0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "linear_velocity_inputs_velocity_0: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity_inputs_velocity_0, out);
    out << "\n";
  }

  // member: linear_velocity_inputs_velocity_0_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "linear_velocity_inputs_velocity_0_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity_inputs_velocity_0_valid, out);
    out << "\n";
  }

  // member: linear_velocity_inputs_velocity_0_transmitted_safely
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "linear_velocity_inputs_velocity_0_transmitted_safely: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity_inputs_velocity_0_transmitted_safely, out);
    out << "\n";
  }

  // member: linear_velocity_inputs_velocity_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "linear_velocity_inputs_velocity_1: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity_inputs_velocity_1, out);
    out << "\n";
  }

  // member: linear_velocity_inputs_velocity_1_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "linear_velocity_inputs_velocity_1_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity_inputs_velocity_1_valid, out);
    out << "\n";
  }

  // member: linear_velocity_inputs_velocity_1_transmitted_safely
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "linear_velocity_inputs_velocity_1_transmitted_safely: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity_inputs_velocity_1_transmitted_safely, out);
    out << "\n";
  }

  // member: sleep_mode_input
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sleep_mode_input: ";
    rosidl_generator_traits::value_to_yaml(msg.sleep_mode_input, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ApplicationInputs & msg, bool use_flow_style = false)
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
  const sick_safetyscanners2_interfaces::msg::ApplicationInputs & msg,
  std::ostream & out, size_t indentation = 0)
{
  sick_safetyscanners2_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sick_safetyscanners2_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const sick_safetyscanners2_interfaces::msg::ApplicationInputs & msg)
{
  return sick_safetyscanners2_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sick_safetyscanners2_interfaces::msg::ApplicationInputs>()
{
  return "sick_safetyscanners2_interfaces::msg::ApplicationInputs";
}

template<>
inline const char * name<sick_safetyscanners2_interfaces::msg::ApplicationInputs>()
{
  return "sick_safetyscanners2_interfaces/msg/ApplicationInputs";
}

template<>
struct has_fixed_size<sick_safetyscanners2_interfaces::msg::ApplicationInputs>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<sick_safetyscanners2_interfaces::msg::ApplicationInputs>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<sick_safetyscanners2_interfaces::msg::ApplicationInputs>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_INPUTS__TRAITS_HPP_
