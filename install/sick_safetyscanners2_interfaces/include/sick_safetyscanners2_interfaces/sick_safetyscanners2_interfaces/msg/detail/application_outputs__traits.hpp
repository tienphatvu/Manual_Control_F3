// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/ApplicationOutputs.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_OUTPUTS__TRAITS_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_OUTPUTS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sick_safetyscanners2_interfaces/msg/detail/application_outputs__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace sick_safetyscanners2_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ApplicationOutputs & msg,
  std::ostream & out)
{
  out << "{";
  // member: evaluation_path_outputs_eval_out
  {
    if (msg.evaluation_path_outputs_eval_out.size() == 0) {
      out << "evaluation_path_outputs_eval_out: []";
    } else {
      out << "evaluation_path_outputs_eval_out: [";
      size_t pending_items = msg.evaluation_path_outputs_eval_out.size();
      for (auto item : msg.evaluation_path_outputs_eval_out) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: evaluation_path_outputs_is_safe
  {
    if (msg.evaluation_path_outputs_is_safe.size() == 0) {
      out << "evaluation_path_outputs_is_safe: []";
    } else {
      out << "evaluation_path_outputs_is_safe: [";
      size_t pending_items = msg.evaluation_path_outputs_is_safe.size();
      for (auto item : msg.evaluation_path_outputs_is_safe) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: evaluation_path_outputs_is_valid
  {
    if (msg.evaluation_path_outputs_is_valid.size() == 0) {
      out << "evaluation_path_outputs_is_valid: []";
    } else {
      out << "evaluation_path_outputs_is_valid: [";
      size_t pending_items = msg.evaluation_path_outputs_is_valid.size();
      for (auto item : msg.evaluation_path_outputs_is_valid) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: monitoring_case_number_outputs
  {
    if (msg.monitoring_case_number_outputs.size() == 0) {
      out << "monitoring_case_number_outputs: []";
    } else {
      out << "monitoring_case_number_outputs: [";
      size_t pending_items = msg.monitoring_case_number_outputs.size();
      for (auto item : msg.monitoring_case_number_outputs) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: monitoring_case_number_outputs_flags
  {
    if (msg.monitoring_case_number_outputs_flags.size() == 0) {
      out << "monitoring_case_number_outputs_flags: []";
    } else {
      out << "monitoring_case_number_outputs_flags: [";
      size_t pending_items = msg.monitoring_case_number_outputs_flags.size();
      for (auto item : msg.monitoring_case_number_outputs_flags) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: sleep_mode_output
  {
    out << "sleep_mode_output: ";
    rosidl_generator_traits::value_to_yaml(msg.sleep_mode_output, out);
    out << ", ";
  }

  // member: sleep_mode_output_valid
  {
    out << "sleep_mode_output_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.sleep_mode_output_valid, out);
    out << ", ";
  }

  // member: error_flag_contamination_warning
  {
    out << "error_flag_contamination_warning: ";
    rosidl_generator_traits::value_to_yaml(msg.error_flag_contamination_warning, out);
    out << ", ";
  }

  // member: error_flag_contamination_error
  {
    out << "error_flag_contamination_error: ";
    rosidl_generator_traits::value_to_yaml(msg.error_flag_contamination_error, out);
    out << ", ";
  }

  // member: error_flag_manipulation_error
  {
    out << "error_flag_manipulation_error: ";
    rosidl_generator_traits::value_to_yaml(msg.error_flag_manipulation_error, out);
    out << ", ";
  }

  // member: error_flag_glare
  {
    out << "error_flag_glare: ";
    rosidl_generator_traits::value_to_yaml(msg.error_flag_glare, out);
    out << ", ";
  }

  // member: error_flag_reference_contour_intruded
  {
    out << "error_flag_reference_contour_intruded: ";
    rosidl_generator_traits::value_to_yaml(msg.error_flag_reference_contour_intruded, out);
    out << ", ";
  }

  // member: error_flag_critical_error
  {
    out << "error_flag_critical_error: ";
    rosidl_generator_traits::value_to_yaml(msg.error_flag_critical_error, out);
    out << ", ";
  }

  // member: error_flags_are_valid
  {
    out << "error_flags_are_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.error_flags_are_valid, out);
    out << ", ";
  }

  // member: linear_velocity_outputs_velocity_0
  {
    out << "linear_velocity_outputs_velocity_0: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity_outputs_velocity_0, out);
    out << ", ";
  }

  // member: linear_velocity_outputs_velocity_0_valid
  {
    out << "linear_velocity_outputs_velocity_0_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity_outputs_velocity_0_valid, out);
    out << ", ";
  }

  // member: linear_velocity_outputs_velocity_0_transmitted_safely
  {
    out << "linear_velocity_outputs_velocity_0_transmitted_safely: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity_outputs_velocity_0_transmitted_safely, out);
    out << ", ";
  }

  // member: linear_velocity_outputs_velocity_1
  {
    out << "linear_velocity_outputs_velocity_1: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity_outputs_velocity_1, out);
    out << ", ";
  }

  // member: linear_velocity_outputs_velocity_1_valid
  {
    out << "linear_velocity_outputs_velocity_1_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity_outputs_velocity_1_valid, out);
    out << ", ";
  }

  // member: linear_velocity_outputs_velocity_1_transmitted_safely
  {
    out << "linear_velocity_outputs_velocity_1_transmitted_safely: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity_outputs_velocity_1_transmitted_safely, out);
    out << ", ";
  }

  // member: resulting_velocity
  {
    if (msg.resulting_velocity.size() == 0) {
      out << "resulting_velocity: []";
    } else {
      out << "resulting_velocity: [";
      size_t pending_items = msg.resulting_velocity.size();
      for (auto item : msg.resulting_velocity) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: resulting_velocity_flags
  {
    if (msg.resulting_velocity_flags.size() == 0) {
      out << "resulting_velocity_flags: []";
    } else {
      out << "resulting_velocity_flags: [";
      size_t pending_items = msg.resulting_velocity_flags.size();
      for (auto item : msg.resulting_velocity_flags) {
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
  const ApplicationOutputs & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: evaluation_path_outputs_eval_out
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.evaluation_path_outputs_eval_out.size() == 0) {
      out << "evaluation_path_outputs_eval_out: []\n";
    } else {
      out << "evaluation_path_outputs_eval_out:\n";
      for (auto item : msg.evaluation_path_outputs_eval_out) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: evaluation_path_outputs_is_safe
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.evaluation_path_outputs_is_safe.size() == 0) {
      out << "evaluation_path_outputs_is_safe: []\n";
    } else {
      out << "evaluation_path_outputs_is_safe:\n";
      for (auto item : msg.evaluation_path_outputs_is_safe) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: evaluation_path_outputs_is_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.evaluation_path_outputs_is_valid.size() == 0) {
      out << "evaluation_path_outputs_is_valid: []\n";
    } else {
      out << "evaluation_path_outputs_is_valid:\n";
      for (auto item : msg.evaluation_path_outputs_is_valid) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: monitoring_case_number_outputs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.monitoring_case_number_outputs.size() == 0) {
      out << "monitoring_case_number_outputs: []\n";
    } else {
      out << "monitoring_case_number_outputs:\n";
      for (auto item : msg.monitoring_case_number_outputs) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: monitoring_case_number_outputs_flags
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.monitoring_case_number_outputs_flags.size() == 0) {
      out << "monitoring_case_number_outputs_flags: []\n";
    } else {
      out << "monitoring_case_number_outputs_flags:\n";
      for (auto item : msg.monitoring_case_number_outputs_flags) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: sleep_mode_output
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sleep_mode_output: ";
    rosidl_generator_traits::value_to_yaml(msg.sleep_mode_output, out);
    out << "\n";
  }

  // member: sleep_mode_output_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sleep_mode_output_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.sleep_mode_output_valid, out);
    out << "\n";
  }

  // member: error_flag_contamination_warning
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_flag_contamination_warning: ";
    rosidl_generator_traits::value_to_yaml(msg.error_flag_contamination_warning, out);
    out << "\n";
  }

  // member: error_flag_contamination_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_flag_contamination_error: ";
    rosidl_generator_traits::value_to_yaml(msg.error_flag_contamination_error, out);
    out << "\n";
  }

  // member: error_flag_manipulation_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_flag_manipulation_error: ";
    rosidl_generator_traits::value_to_yaml(msg.error_flag_manipulation_error, out);
    out << "\n";
  }

  // member: error_flag_glare
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_flag_glare: ";
    rosidl_generator_traits::value_to_yaml(msg.error_flag_glare, out);
    out << "\n";
  }

  // member: error_flag_reference_contour_intruded
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_flag_reference_contour_intruded: ";
    rosidl_generator_traits::value_to_yaml(msg.error_flag_reference_contour_intruded, out);
    out << "\n";
  }

  // member: error_flag_critical_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_flag_critical_error: ";
    rosidl_generator_traits::value_to_yaml(msg.error_flag_critical_error, out);
    out << "\n";
  }

  // member: error_flags_are_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_flags_are_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.error_flags_are_valid, out);
    out << "\n";
  }

  // member: linear_velocity_outputs_velocity_0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "linear_velocity_outputs_velocity_0: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity_outputs_velocity_0, out);
    out << "\n";
  }

  // member: linear_velocity_outputs_velocity_0_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "linear_velocity_outputs_velocity_0_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity_outputs_velocity_0_valid, out);
    out << "\n";
  }

  // member: linear_velocity_outputs_velocity_0_transmitted_safely
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "linear_velocity_outputs_velocity_0_transmitted_safely: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity_outputs_velocity_0_transmitted_safely, out);
    out << "\n";
  }

  // member: linear_velocity_outputs_velocity_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "linear_velocity_outputs_velocity_1: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity_outputs_velocity_1, out);
    out << "\n";
  }

  // member: linear_velocity_outputs_velocity_1_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "linear_velocity_outputs_velocity_1_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity_outputs_velocity_1_valid, out);
    out << "\n";
  }

  // member: linear_velocity_outputs_velocity_1_transmitted_safely
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "linear_velocity_outputs_velocity_1_transmitted_safely: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity_outputs_velocity_1_transmitted_safely, out);
    out << "\n";
  }

  // member: resulting_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.resulting_velocity.size() == 0) {
      out << "resulting_velocity: []\n";
    } else {
      out << "resulting_velocity:\n";
      for (auto item : msg.resulting_velocity) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: resulting_velocity_flags
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.resulting_velocity_flags.size() == 0) {
      out << "resulting_velocity_flags: []\n";
    } else {
      out << "resulting_velocity_flags:\n";
      for (auto item : msg.resulting_velocity_flags) {
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

inline std::string to_yaml(const ApplicationOutputs & msg, bool use_flow_style = false)
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
  const sick_safetyscanners2_interfaces::msg::ApplicationOutputs & msg,
  std::ostream & out, size_t indentation = 0)
{
  sick_safetyscanners2_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sick_safetyscanners2_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const sick_safetyscanners2_interfaces::msg::ApplicationOutputs & msg)
{
  return sick_safetyscanners2_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sick_safetyscanners2_interfaces::msg::ApplicationOutputs>()
{
  return "sick_safetyscanners2_interfaces::msg::ApplicationOutputs";
}

template<>
inline const char * name<sick_safetyscanners2_interfaces::msg::ApplicationOutputs>()
{
  return "sick_safetyscanners2_interfaces/msg/ApplicationOutputs";
}

template<>
struct has_fixed_size<sick_safetyscanners2_interfaces::msg::ApplicationOutputs>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<sick_safetyscanners2_interfaces::msg::ApplicationOutputs>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<sick_safetyscanners2_interfaces::msg::ApplicationOutputs>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_OUTPUTS__TRAITS_HPP_
