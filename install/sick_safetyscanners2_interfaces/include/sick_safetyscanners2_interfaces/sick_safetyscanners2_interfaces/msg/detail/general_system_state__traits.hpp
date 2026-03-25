// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/GeneralSystemState.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__GENERAL_SYSTEM_STATE__TRAITS_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__GENERAL_SYSTEM_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sick_safetyscanners2_interfaces/msg/detail/general_system_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace sick_safetyscanners2_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const GeneralSystemState & msg,
  std::ostream & out)
{
  out << "{";
  // member: run_mode_active
  {
    out << "run_mode_active: ";
    rosidl_generator_traits::value_to_yaml(msg.run_mode_active, out);
    out << ", ";
  }

  // member: standby_mode_active
  {
    out << "standby_mode_active: ";
    rosidl_generator_traits::value_to_yaml(msg.standby_mode_active, out);
    out << ", ";
  }

  // member: contamination_warning
  {
    out << "contamination_warning: ";
    rosidl_generator_traits::value_to_yaml(msg.contamination_warning, out);
    out << ", ";
  }

  // member: contamination_error
  {
    out << "contamination_error: ";
    rosidl_generator_traits::value_to_yaml(msg.contamination_error, out);
    out << ", ";
  }

  // member: reference_contour_status
  {
    out << "reference_contour_status: ";
    rosidl_generator_traits::value_to_yaml(msg.reference_contour_status, out);
    out << ", ";
  }

  // member: manipulation_status
  {
    out << "manipulation_status: ";
    rosidl_generator_traits::value_to_yaml(msg.manipulation_status, out);
    out << ", ";
  }

  // member: safe_cut_off_path
  {
    if (msg.safe_cut_off_path.size() == 0) {
      out << "safe_cut_off_path: []";
    } else {
      out << "safe_cut_off_path: [";
      size_t pending_items = msg.safe_cut_off_path.size();
      for (auto item : msg.safe_cut_off_path) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: non_safe_cut_off_path
  {
    if (msg.non_safe_cut_off_path.size() == 0) {
      out << "non_safe_cut_off_path: []";
    } else {
      out << "non_safe_cut_off_path: [";
      size_t pending_items = msg.non_safe_cut_off_path.size();
      for (auto item : msg.non_safe_cut_off_path) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: reset_required_cut_off_path
  {
    if (msg.reset_required_cut_off_path.size() == 0) {
      out << "reset_required_cut_off_path: []";
    } else {
      out << "reset_required_cut_off_path: [";
      size_t pending_items = msg.reset_required_cut_off_path.size();
      for (auto item : msg.reset_required_cut_off_path) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: current_monitoring_case_no_table_1
  {
    out << "current_monitoring_case_no_table_1: ";
    rosidl_generator_traits::value_to_yaml(msg.current_monitoring_case_no_table_1, out);
    out << ", ";
  }

  // member: current_monitoring_case_no_table_2
  {
    out << "current_monitoring_case_no_table_2: ";
    rosidl_generator_traits::value_to_yaml(msg.current_monitoring_case_no_table_2, out);
    out << ", ";
  }

  // member: current_monitoring_case_no_table_3
  {
    out << "current_monitoring_case_no_table_3: ";
    rosidl_generator_traits::value_to_yaml(msg.current_monitoring_case_no_table_3, out);
    out << ", ";
  }

  // member: current_monitoring_case_no_table_4
  {
    out << "current_monitoring_case_no_table_4: ";
    rosidl_generator_traits::value_to_yaml(msg.current_monitoring_case_no_table_4, out);
    out << ", ";
  }

  // member: application_error
  {
    out << "application_error: ";
    rosidl_generator_traits::value_to_yaml(msg.application_error, out);
    out << ", ";
  }

  // member: device_error
  {
    out << "device_error: ";
    rosidl_generator_traits::value_to_yaml(msg.device_error, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GeneralSystemState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: run_mode_active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "run_mode_active: ";
    rosidl_generator_traits::value_to_yaml(msg.run_mode_active, out);
    out << "\n";
  }

  // member: standby_mode_active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "standby_mode_active: ";
    rosidl_generator_traits::value_to_yaml(msg.standby_mode_active, out);
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

  // member: contamination_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "contamination_error: ";
    rosidl_generator_traits::value_to_yaml(msg.contamination_error, out);
    out << "\n";
  }

  // member: reference_contour_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reference_contour_status: ";
    rosidl_generator_traits::value_to_yaml(msg.reference_contour_status, out);
    out << "\n";
  }

  // member: manipulation_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "manipulation_status: ";
    rosidl_generator_traits::value_to_yaml(msg.manipulation_status, out);
    out << "\n";
  }

  // member: safe_cut_off_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.safe_cut_off_path.size() == 0) {
      out << "safe_cut_off_path: []\n";
    } else {
      out << "safe_cut_off_path:\n";
      for (auto item : msg.safe_cut_off_path) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: non_safe_cut_off_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.non_safe_cut_off_path.size() == 0) {
      out << "non_safe_cut_off_path: []\n";
    } else {
      out << "non_safe_cut_off_path:\n";
      for (auto item : msg.non_safe_cut_off_path) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: reset_required_cut_off_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.reset_required_cut_off_path.size() == 0) {
      out << "reset_required_cut_off_path: []\n";
    } else {
      out << "reset_required_cut_off_path:\n";
      for (auto item : msg.reset_required_cut_off_path) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: current_monitoring_case_no_table_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_monitoring_case_no_table_1: ";
    rosidl_generator_traits::value_to_yaml(msg.current_monitoring_case_no_table_1, out);
    out << "\n";
  }

  // member: current_monitoring_case_no_table_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_monitoring_case_no_table_2: ";
    rosidl_generator_traits::value_to_yaml(msg.current_monitoring_case_no_table_2, out);
    out << "\n";
  }

  // member: current_monitoring_case_no_table_3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_monitoring_case_no_table_3: ";
    rosidl_generator_traits::value_to_yaml(msg.current_monitoring_case_no_table_3, out);
    out << "\n";
  }

  // member: current_monitoring_case_no_table_4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_monitoring_case_no_table_4: ";
    rosidl_generator_traits::value_to_yaml(msg.current_monitoring_case_no_table_4, out);
    out << "\n";
  }

  // member: application_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "application_error: ";
    rosidl_generator_traits::value_to_yaml(msg.application_error, out);
    out << "\n";
  }

  // member: device_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "device_error: ";
    rosidl_generator_traits::value_to_yaml(msg.device_error, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GeneralSystemState & msg, bool use_flow_style = false)
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
  const sick_safetyscanners2_interfaces::msg::GeneralSystemState & msg,
  std::ostream & out, size_t indentation = 0)
{
  sick_safetyscanners2_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sick_safetyscanners2_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const sick_safetyscanners2_interfaces::msg::GeneralSystemState & msg)
{
  return sick_safetyscanners2_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sick_safetyscanners2_interfaces::msg::GeneralSystemState>()
{
  return "sick_safetyscanners2_interfaces::msg::GeneralSystemState";
}

template<>
inline const char * name<sick_safetyscanners2_interfaces::msg::GeneralSystemState>()
{
  return "sick_safetyscanners2_interfaces/msg/GeneralSystemState";
}

template<>
struct has_fixed_size<sick_safetyscanners2_interfaces::msg::GeneralSystemState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<sick_safetyscanners2_interfaces::msg::GeneralSystemState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<sick_safetyscanners2_interfaces::msg::GeneralSystemState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__GENERAL_SYSTEM_STATE__TRAITS_HPP_
