// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/ApplicationOutputs.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_OUTPUTS__BUILDER_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_OUTPUTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sick_safetyscanners2_interfaces/msg/detail/application_outputs__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sick_safetyscanners2_interfaces
{

namespace msg
{

namespace builder
{

class Init_ApplicationOutputs_resulting_velocity_flags
{
public:
  explicit Init_ApplicationOutputs_resulting_velocity_flags(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs & msg)
  : msg_(msg)
  {}
  ::sick_safetyscanners2_interfaces::msg::ApplicationOutputs resulting_velocity_flags(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs::_resulting_velocity_flags_type arg)
  {
    msg_.resulting_velocity_flags = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationOutputs msg_;
};

class Init_ApplicationOutputs_resulting_velocity
{
public:
  explicit Init_ApplicationOutputs_resulting_velocity(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs & msg)
  : msg_(msg)
  {}
  Init_ApplicationOutputs_resulting_velocity_flags resulting_velocity(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs::_resulting_velocity_type arg)
  {
    msg_.resulting_velocity = std::move(arg);
    return Init_ApplicationOutputs_resulting_velocity_flags(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationOutputs msg_;
};

class Init_ApplicationOutputs_linear_velocity_outputs_velocity_1_transmitted_safely
{
public:
  explicit Init_ApplicationOutputs_linear_velocity_outputs_velocity_1_transmitted_safely(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs & msg)
  : msg_(msg)
  {}
  Init_ApplicationOutputs_resulting_velocity linear_velocity_outputs_velocity_1_transmitted_safely(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs::_linear_velocity_outputs_velocity_1_transmitted_safely_type arg)
  {
    msg_.linear_velocity_outputs_velocity_1_transmitted_safely = std::move(arg);
    return Init_ApplicationOutputs_resulting_velocity(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationOutputs msg_;
};

class Init_ApplicationOutputs_linear_velocity_outputs_velocity_1_valid
{
public:
  explicit Init_ApplicationOutputs_linear_velocity_outputs_velocity_1_valid(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs & msg)
  : msg_(msg)
  {}
  Init_ApplicationOutputs_linear_velocity_outputs_velocity_1_transmitted_safely linear_velocity_outputs_velocity_1_valid(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs::_linear_velocity_outputs_velocity_1_valid_type arg)
  {
    msg_.linear_velocity_outputs_velocity_1_valid = std::move(arg);
    return Init_ApplicationOutputs_linear_velocity_outputs_velocity_1_transmitted_safely(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationOutputs msg_;
};

class Init_ApplicationOutputs_linear_velocity_outputs_velocity_1
{
public:
  explicit Init_ApplicationOutputs_linear_velocity_outputs_velocity_1(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs & msg)
  : msg_(msg)
  {}
  Init_ApplicationOutputs_linear_velocity_outputs_velocity_1_valid linear_velocity_outputs_velocity_1(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs::_linear_velocity_outputs_velocity_1_type arg)
  {
    msg_.linear_velocity_outputs_velocity_1 = std::move(arg);
    return Init_ApplicationOutputs_linear_velocity_outputs_velocity_1_valid(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationOutputs msg_;
};

class Init_ApplicationOutputs_linear_velocity_outputs_velocity_0_transmitted_safely
{
public:
  explicit Init_ApplicationOutputs_linear_velocity_outputs_velocity_0_transmitted_safely(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs & msg)
  : msg_(msg)
  {}
  Init_ApplicationOutputs_linear_velocity_outputs_velocity_1 linear_velocity_outputs_velocity_0_transmitted_safely(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs::_linear_velocity_outputs_velocity_0_transmitted_safely_type arg)
  {
    msg_.linear_velocity_outputs_velocity_0_transmitted_safely = std::move(arg);
    return Init_ApplicationOutputs_linear_velocity_outputs_velocity_1(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationOutputs msg_;
};

class Init_ApplicationOutputs_linear_velocity_outputs_velocity_0_valid
{
public:
  explicit Init_ApplicationOutputs_linear_velocity_outputs_velocity_0_valid(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs & msg)
  : msg_(msg)
  {}
  Init_ApplicationOutputs_linear_velocity_outputs_velocity_0_transmitted_safely linear_velocity_outputs_velocity_0_valid(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs::_linear_velocity_outputs_velocity_0_valid_type arg)
  {
    msg_.linear_velocity_outputs_velocity_0_valid = std::move(arg);
    return Init_ApplicationOutputs_linear_velocity_outputs_velocity_0_transmitted_safely(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationOutputs msg_;
};

class Init_ApplicationOutputs_linear_velocity_outputs_velocity_0
{
public:
  explicit Init_ApplicationOutputs_linear_velocity_outputs_velocity_0(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs & msg)
  : msg_(msg)
  {}
  Init_ApplicationOutputs_linear_velocity_outputs_velocity_0_valid linear_velocity_outputs_velocity_0(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs::_linear_velocity_outputs_velocity_0_type arg)
  {
    msg_.linear_velocity_outputs_velocity_0 = std::move(arg);
    return Init_ApplicationOutputs_linear_velocity_outputs_velocity_0_valid(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationOutputs msg_;
};

class Init_ApplicationOutputs_error_flags_are_valid
{
public:
  explicit Init_ApplicationOutputs_error_flags_are_valid(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs & msg)
  : msg_(msg)
  {}
  Init_ApplicationOutputs_linear_velocity_outputs_velocity_0 error_flags_are_valid(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs::_error_flags_are_valid_type arg)
  {
    msg_.error_flags_are_valid = std::move(arg);
    return Init_ApplicationOutputs_linear_velocity_outputs_velocity_0(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationOutputs msg_;
};

class Init_ApplicationOutputs_error_flag_critical_error
{
public:
  explicit Init_ApplicationOutputs_error_flag_critical_error(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs & msg)
  : msg_(msg)
  {}
  Init_ApplicationOutputs_error_flags_are_valid error_flag_critical_error(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs::_error_flag_critical_error_type arg)
  {
    msg_.error_flag_critical_error = std::move(arg);
    return Init_ApplicationOutputs_error_flags_are_valid(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationOutputs msg_;
};

class Init_ApplicationOutputs_error_flag_reference_contour_intruded
{
public:
  explicit Init_ApplicationOutputs_error_flag_reference_contour_intruded(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs & msg)
  : msg_(msg)
  {}
  Init_ApplicationOutputs_error_flag_critical_error error_flag_reference_contour_intruded(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs::_error_flag_reference_contour_intruded_type arg)
  {
    msg_.error_flag_reference_contour_intruded = std::move(arg);
    return Init_ApplicationOutputs_error_flag_critical_error(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationOutputs msg_;
};

class Init_ApplicationOutputs_error_flag_glare
{
public:
  explicit Init_ApplicationOutputs_error_flag_glare(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs & msg)
  : msg_(msg)
  {}
  Init_ApplicationOutputs_error_flag_reference_contour_intruded error_flag_glare(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs::_error_flag_glare_type arg)
  {
    msg_.error_flag_glare = std::move(arg);
    return Init_ApplicationOutputs_error_flag_reference_contour_intruded(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationOutputs msg_;
};

class Init_ApplicationOutputs_error_flag_manipulation_error
{
public:
  explicit Init_ApplicationOutputs_error_flag_manipulation_error(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs & msg)
  : msg_(msg)
  {}
  Init_ApplicationOutputs_error_flag_glare error_flag_manipulation_error(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs::_error_flag_manipulation_error_type arg)
  {
    msg_.error_flag_manipulation_error = std::move(arg);
    return Init_ApplicationOutputs_error_flag_glare(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationOutputs msg_;
};

class Init_ApplicationOutputs_error_flag_contamination_error
{
public:
  explicit Init_ApplicationOutputs_error_flag_contamination_error(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs & msg)
  : msg_(msg)
  {}
  Init_ApplicationOutputs_error_flag_manipulation_error error_flag_contamination_error(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs::_error_flag_contamination_error_type arg)
  {
    msg_.error_flag_contamination_error = std::move(arg);
    return Init_ApplicationOutputs_error_flag_manipulation_error(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationOutputs msg_;
};

class Init_ApplicationOutputs_error_flag_contamination_warning
{
public:
  explicit Init_ApplicationOutputs_error_flag_contamination_warning(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs & msg)
  : msg_(msg)
  {}
  Init_ApplicationOutputs_error_flag_contamination_error error_flag_contamination_warning(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs::_error_flag_contamination_warning_type arg)
  {
    msg_.error_flag_contamination_warning = std::move(arg);
    return Init_ApplicationOutputs_error_flag_contamination_error(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationOutputs msg_;
};

class Init_ApplicationOutputs_sleep_mode_output_valid
{
public:
  explicit Init_ApplicationOutputs_sleep_mode_output_valid(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs & msg)
  : msg_(msg)
  {}
  Init_ApplicationOutputs_error_flag_contamination_warning sleep_mode_output_valid(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs::_sleep_mode_output_valid_type arg)
  {
    msg_.sleep_mode_output_valid = std::move(arg);
    return Init_ApplicationOutputs_error_flag_contamination_warning(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationOutputs msg_;
};

class Init_ApplicationOutputs_sleep_mode_output
{
public:
  explicit Init_ApplicationOutputs_sleep_mode_output(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs & msg)
  : msg_(msg)
  {}
  Init_ApplicationOutputs_sleep_mode_output_valid sleep_mode_output(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs::_sleep_mode_output_type arg)
  {
    msg_.sleep_mode_output = std::move(arg);
    return Init_ApplicationOutputs_sleep_mode_output_valid(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationOutputs msg_;
};

class Init_ApplicationOutputs_monitoring_case_number_outputs_flags
{
public:
  explicit Init_ApplicationOutputs_monitoring_case_number_outputs_flags(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs & msg)
  : msg_(msg)
  {}
  Init_ApplicationOutputs_sleep_mode_output monitoring_case_number_outputs_flags(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs::_monitoring_case_number_outputs_flags_type arg)
  {
    msg_.monitoring_case_number_outputs_flags = std::move(arg);
    return Init_ApplicationOutputs_sleep_mode_output(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationOutputs msg_;
};

class Init_ApplicationOutputs_monitoring_case_number_outputs
{
public:
  explicit Init_ApplicationOutputs_monitoring_case_number_outputs(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs & msg)
  : msg_(msg)
  {}
  Init_ApplicationOutputs_monitoring_case_number_outputs_flags monitoring_case_number_outputs(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs::_monitoring_case_number_outputs_type arg)
  {
    msg_.monitoring_case_number_outputs = std::move(arg);
    return Init_ApplicationOutputs_monitoring_case_number_outputs_flags(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationOutputs msg_;
};

class Init_ApplicationOutputs_evaluation_path_outputs_is_valid
{
public:
  explicit Init_ApplicationOutputs_evaluation_path_outputs_is_valid(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs & msg)
  : msg_(msg)
  {}
  Init_ApplicationOutputs_monitoring_case_number_outputs evaluation_path_outputs_is_valid(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs::_evaluation_path_outputs_is_valid_type arg)
  {
    msg_.evaluation_path_outputs_is_valid = std::move(arg);
    return Init_ApplicationOutputs_monitoring_case_number_outputs(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationOutputs msg_;
};

class Init_ApplicationOutputs_evaluation_path_outputs_is_safe
{
public:
  explicit Init_ApplicationOutputs_evaluation_path_outputs_is_safe(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs & msg)
  : msg_(msg)
  {}
  Init_ApplicationOutputs_evaluation_path_outputs_is_valid evaluation_path_outputs_is_safe(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs::_evaluation_path_outputs_is_safe_type arg)
  {
    msg_.evaluation_path_outputs_is_safe = std::move(arg);
    return Init_ApplicationOutputs_evaluation_path_outputs_is_valid(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationOutputs msg_;
};

class Init_ApplicationOutputs_evaluation_path_outputs_eval_out
{
public:
  Init_ApplicationOutputs_evaluation_path_outputs_eval_out()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ApplicationOutputs_evaluation_path_outputs_is_safe evaluation_path_outputs_eval_out(::sick_safetyscanners2_interfaces::msg::ApplicationOutputs::_evaluation_path_outputs_eval_out_type arg)
  {
    msg_.evaluation_path_outputs_eval_out = std::move(arg);
    return Init_ApplicationOutputs_evaluation_path_outputs_is_safe(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationOutputs msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sick_safetyscanners2_interfaces::msg::ApplicationOutputs>()
{
  return sick_safetyscanners2_interfaces::msg::builder::Init_ApplicationOutputs_evaluation_path_outputs_eval_out();
}

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_OUTPUTS__BUILDER_HPP_
