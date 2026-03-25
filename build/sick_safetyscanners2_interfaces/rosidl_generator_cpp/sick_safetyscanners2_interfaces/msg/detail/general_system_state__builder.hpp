// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/GeneralSystemState.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__GENERAL_SYSTEM_STATE__BUILDER_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__GENERAL_SYSTEM_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sick_safetyscanners2_interfaces/msg/detail/general_system_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sick_safetyscanners2_interfaces
{

namespace msg
{

namespace builder
{

class Init_GeneralSystemState_device_error
{
public:
  explicit Init_GeneralSystemState_device_error(::sick_safetyscanners2_interfaces::msg::GeneralSystemState & msg)
  : msg_(msg)
  {}
  ::sick_safetyscanners2_interfaces::msg::GeneralSystemState device_error(::sick_safetyscanners2_interfaces::msg::GeneralSystemState::_device_error_type arg)
  {
    msg_.device_error = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::GeneralSystemState msg_;
};

class Init_GeneralSystemState_application_error
{
public:
  explicit Init_GeneralSystemState_application_error(::sick_safetyscanners2_interfaces::msg::GeneralSystemState & msg)
  : msg_(msg)
  {}
  Init_GeneralSystemState_device_error application_error(::sick_safetyscanners2_interfaces::msg::GeneralSystemState::_application_error_type arg)
  {
    msg_.application_error = std::move(arg);
    return Init_GeneralSystemState_device_error(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::GeneralSystemState msg_;
};

class Init_GeneralSystemState_current_monitoring_case_no_table_4
{
public:
  explicit Init_GeneralSystemState_current_monitoring_case_no_table_4(::sick_safetyscanners2_interfaces::msg::GeneralSystemState & msg)
  : msg_(msg)
  {}
  Init_GeneralSystemState_application_error current_monitoring_case_no_table_4(::sick_safetyscanners2_interfaces::msg::GeneralSystemState::_current_monitoring_case_no_table_4_type arg)
  {
    msg_.current_monitoring_case_no_table_4 = std::move(arg);
    return Init_GeneralSystemState_application_error(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::GeneralSystemState msg_;
};

class Init_GeneralSystemState_current_monitoring_case_no_table_3
{
public:
  explicit Init_GeneralSystemState_current_monitoring_case_no_table_3(::sick_safetyscanners2_interfaces::msg::GeneralSystemState & msg)
  : msg_(msg)
  {}
  Init_GeneralSystemState_current_monitoring_case_no_table_4 current_monitoring_case_no_table_3(::sick_safetyscanners2_interfaces::msg::GeneralSystemState::_current_monitoring_case_no_table_3_type arg)
  {
    msg_.current_monitoring_case_no_table_3 = std::move(arg);
    return Init_GeneralSystemState_current_monitoring_case_no_table_4(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::GeneralSystemState msg_;
};

class Init_GeneralSystemState_current_monitoring_case_no_table_2
{
public:
  explicit Init_GeneralSystemState_current_monitoring_case_no_table_2(::sick_safetyscanners2_interfaces::msg::GeneralSystemState & msg)
  : msg_(msg)
  {}
  Init_GeneralSystemState_current_monitoring_case_no_table_3 current_monitoring_case_no_table_2(::sick_safetyscanners2_interfaces::msg::GeneralSystemState::_current_monitoring_case_no_table_2_type arg)
  {
    msg_.current_monitoring_case_no_table_2 = std::move(arg);
    return Init_GeneralSystemState_current_monitoring_case_no_table_3(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::GeneralSystemState msg_;
};

class Init_GeneralSystemState_current_monitoring_case_no_table_1
{
public:
  explicit Init_GeneralSystemState_current_monitoring_case_no_table_1(::sick_safetyscanners2_interfaces::msg::GeneralSystemState & msg)
  : msg_(msg)
  {}
  Init_GeneralSystemState_current_monitoring_case_no_table_2 current_monitoring_case_no_table_1(::sick_safetyscanners2_interfaces::msg::GeneralSystemState::_current_monitoring_case_no_table_1_type arg)
  {
    msg_.current_monitoring_case_no_table_1 = std::move(arg);
    return Init_GeneralSystemState_current_monitoring_case_no_table_2(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::GeneralSystemState msg_;
};

class Init_GeneralSystemState_reset_required_cut_off_path
{
public:
  explicit Init_GeneralSystemState_reset_required_cut_off_path(::sick_safetyscanners2_interfaces::msg::GeneralSystemState & msg)
  : msg_(msg)
  {}
  Init_GeneralSystemState_current_monitoring_case_no_table_1 reset_required_cut_off_path(::sick_safetyscanners2_interfaces::msg::GeneralSystemState::_reset_required_cut_off_path_type arg)
  {
    msg_.reset_required_cut_off_path = std::move(arg);
    return Init_GeneralSystemState_current_monitoring_case_no_table_1(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::GeneralSystemState msg_;
};

class Init_GeneralSystemState_non_safe_cut_off_path
{
public:
  explicit Init_GeneralSystemState_non_safe_cut_off_path(::sick_safetyscanners2_interfaces::msg::GeneralSystemState & msg)
  : msg_(msg)
  {}
  Init_GeneralSystemState_reset_required_cut_off_path non_safe_cut_off_path(::sick_safetyscanners2_interfaces::msg::GeneralSystemState::_non_safe_cut_off_path_type arg)
  {
    msg_.non_safe_cut_off_path = std::move(arg);
    return Init_GeneralSystemState_reset_required_cut_off_path(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::GeneralSystemState msg_;
};

class Init_GeneralSystemState_safe_cut_off_path
{
public:
  explicit Init_GeneralSystemState_safe_cut_off_path(::sick_safetyscanners2_interfaces::msg::GeneralSystemState & msg)
  : msg_(msg)
  {}
  Init_GeneralSystemState_non_safe_cut_off_path safe_cut_off_path(::sick_safetyscanners2_interfaces::msg::GeneralSystemState::_safe_cut_off_path_type arg)
  {
    msg_.safe_cut_off_path = std::move(arg);
    return Init_GeneralSystemState_non_safe_cut_off_path(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::GeneralSystemState msg_;
};

class Init_GeneralSystemState_manipulation_status
{
public:
  explicit Init_GeneralSystemState_manipulation_status(::sick_safetyscanners2_interfaces::msg::GeneralSystemState & msg)
  : msg_(msg)
  {}
  Init_GeneralSystemState_safe_cut_off_path manipulation_status(::sick_safetyscanners2_interfaces::msg::GeneralSystemState::_manipulation_status_type arg)
  {
    msg_.manipulation_status = std::move(arg);
    return Init_GeneralSystemState_safe_cut_off_path(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::GeneralSystemState msg_;
};

class Init_GeneralSystemState_reference_contour_status
{
public:
  explicit Init_GeneralSystemState_reference_contour_status(::sick_safetyscanners2_interfaces::msg::GeneralSystemState & msg)
  : msg_(msg)
  {}
  Init_GeneralSystemState_manipulation_status reference_contour_status(::sick_safetyscanners2_interfaces::msg::GeneralSystemState::_reference_contour_status_type arg)
  {
    msg_.reference_contour_status = std::move(arg);
    return Init_GeneralSystemState_manipulation_status(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::GeneralSystemState msg_;
};

class Init_GeneralSystemState_contamination_error
{
public:
  explicit Init_GeneralSystemState_contamination_error(::sick_safetyscanners2_interfaces::msg::GeneralSystemState & msg)
  : msg_(msg)
  {}
  Init_GeneralSystemState_reference_contour_status contamination_error(::sick_safetyscanners2_interfaces::msg::GeneralSystemState::_contamination_error_type arg)
  {
    msg_.contamination_error = std::move(arg);
    return Init_GeneralSystemState_reference_contour_status(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::GeneralSystemState msg_;
};

class Init_GeneralSystemState_contamination_warning
{
public:
  explicit Init_GeneralSystemState_contamination_warning(::sick_safetyscanners2_interfaces::msg::GeneralSystemState & msg)
  : msg_(msg)
  {}
  Init_GeneralSystemState_contamination_error contamination_warning(::sick_safetyscanners2_interfaces::msg::GeneralSystemState::_contamination_warning_type arg)
  {
    msg_.contamination_warning = std::move(arg);
    return Init_GeneralSystemState_contamination_error(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::GeneralSystemState msg_;
};

class Init_GeneralSystemState_standby_mode_active
{
public:
  explicit Init_GeneralSystemState_standby_mode_active(::sick_safetyscanners2_interfaces::msg::GeneralSystemState & msg)
  : msg_(msg)
  {}
  Init_GeneralSystemState_contamination_warning standby_mode_active(::sick_safetyscanners2_interfaces::msg::GeneralSystemState::_standby_mode_active_type arg)
  {
    msg_.standby_mode_active = std::move(arg);
    return Init_GeneralSystemState_contamination_warning(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::GeneralSystemState msg_;
};

class Init_GeneralSystemState_run_mode_active
{
public:
  Init_GeneralSystemState_run_mode_active()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GeneralSystemState_standby_mode_active run_mode_active(::sick_safetyscanners2_interfaces::msg::GeneralSystemState::_run_mode_active_type arg)
  {
    msg_.run_mode_active = std::move(arg);
    return Init_GeneralSystemState_standby_mode_active(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::GeneralSystemState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sick_safetyscanners2_interfaces::msg::GeneralSystemState>()
{
  return sick_safetyscanners2_interfaces::msg::builder::Init_GeneralSystemState_run_mode_active();
}

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__GENERAL_SYSTEM_STATE__BUILDER_HPP_
