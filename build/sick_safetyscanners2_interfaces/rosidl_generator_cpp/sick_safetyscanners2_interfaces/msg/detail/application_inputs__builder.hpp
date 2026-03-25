// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/ApplicationInputs.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_INPUTS__BUILDER_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_INPUTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sick_safetyscanners2_interfaces/msg/detail/application_inputs__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sick_safetyscanners2_interfaces
{

namespace msg
{

namespace builder
{

class Init_ApplicationInputs_sleep_mode_input
{
public:
  explicit Init_ApplicationInputs_sleep_mode_input(::sick_safetyscanners2_interfaces::msg::ApplicationInputs & msg)
  : msg_(msg)
  {}
  ::sick_safetyscanners2_interfaces::msg::ApplicationInputs sleep_mode_input(::sick_safetyscanners2_interfaces::msg::ApplicationInputs::_sleep_mode_input_type arg)
  {
    msg_.sleep_mode_input = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationInputs msg_;
};

class Init_ApplicationInputs_linear_velocity_inputs_velocity_1_transmitted_safely
{
public:
  explicit Init_ApplicationInputs_linear_velocity_inputs_velocity_1_transmitted_safely(::sick_safetyscanners2_interfaces::msg::ApplicationInputs & msg)
  : msg_(msg)
  {}
  Init_ApplicationInputs_sleep_mode_input linear_velocity_inputs_velocity_1_transmitted_safely(::sick_safetyscanners2_interfaces::msg::ApplicationInputs::_linear_velocity_inputs_velocity_1_transmitted_safely_type arg)
  {
    msg_.linear_velocity_inputs_velocity_1_transmitted_safely = std::move(arg);
    return Init_ApplicationInputs_sleep_mode_input(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationInputs msg_;
};

class Init_ApplicationInputs_linear_velocity_inputs_velocity_1_valid
{
public:
  explicit Init_ApplicationInputs_linear_velocity_inputs_velocity_1_valid(::sick_safetyscanners2_interfaces::msg::ApplicationInputs & msg)
  : msg_(msg)
  {}
  Init_ApplicationInputs_linear_velocity_inputs_velocity_1_transmitted_safely linear_velocity_inputs_velocity_1_valid(::sick_safetyscanners2_interfaces::msg::ApplicationInputs::_linear_velocity_inputs_velocity_1_valid_type arg)
  {
    msg_.linear_velocity_inputs_velocity_1_valid = std::move(arg);
    return Init_ApplicationInputs_linear_velocity_inputs_velocity_1_transmitted_safely(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationInputs msg_;
};

class Init_ApplicationInputs_linear_velocity_inputs_velocity_1
{
public:
  explicit Init_ApplicationInputs_linear_velocity_inputs_velocity_1(::sick_safetyscanners2_interfaces::msg::ApplicationInputs & msg)
  : msg_(msg)
  {}
  Init_ApplicationInputs_linear_velocity_inputs_velocity_1_valid linear_velocity_inputs_velocity_1(::sick_safetyscanners2_interfaces::msg::ApplicationInputs::_linear_velocity_inputs_velocity_1_type arg)
  {
    msg_.linear_velocity_inputs_velocity_1 = std::move(arg);
    return Init_ApplicationInputs_linear_velocity_inputs_velocity_1_valid(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationInputs msg_;
};

class Init_ApplicationInputs_linear_velocity_inputs_velocity_0_transmitted_safely
{
public:
  explicit Init_ApplicationInputs_linear_velocity_inputs_velocity_0_transmitted_safely(::sick_safetyscanners2_interfaces::msg::ApplicationInputs & msg)
  : msg_(msg)
  {}
  Init_ApplicationInputs_linear_velocity_inputs_velocity_1 linear_velocity_inputs_velocity_0_transmitted_safely(::sick_safetyscanners2_interfaces::msg::ApplicationInputs::_linear_velocity_inputs_velocity_0_transmitted_safely_type arg)
  {
    msg_.linear_velocity_inputs_velocity_0_transmitted_safely = std::move(arg);
    return Init_ApplicationInputs_linear_velocity_inputs_velocity_1(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationInputs msg_;
};

class Init_ApplicationInputs_linear_velocity_inputs_velocity_0_valid
{
public:
  explicit Init_ApplicationInputs_linear_velocity_inputs_velocity_0_valid(::sick_safetyscanners2_interfaces::msg::ApplicationInputs & msg)
  : msg_(msg)
  {}
  Init_ApplicationInputs_linear_velocity_inputs_velocity_0_transmitted_safely linear_velocity_inputs_velocity_0_valid(::sick_safetyscanners2_interfaces::msg::ApplicationInputs::_linear_velocity_inputs_velocity_0_valid_type arg)
  {
    msg_.linear_velocity_inputs_velocity_0_valid = std::move(arg);
    return Init_ApplicationInputs_linear_velocity_inputs_velocity_0_transmitted_safely(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationInputs msg_;
};

class Init_ApplicationInputs_linear_velocity_inputs_velocity_0
{
public:
  explicit Init_ApplicationInputs_linear_velocity_inputs_velocity_0(::sick_safetyscanners2_interfaces::msg::ApplicationInputs & msg)
  : msg_(msg)
  {}
  Init_ApplicationInputs_linear_velocity_inputs_velocity_0_valid linear_velocity_inputs_velocity_0(::sick_safetyscanners2_interfaces::msg::ApplicationInputs::_linear_velocity_inputs_velocity_0_type arg)
  {
    msg_.linear_velocity_inputs_velocity_0 = std::move(arg);
    return Init_ApplicationInputs_linear_velocity_inputs_velocity_0_valid(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationInputs msg_;
};

class Init_ApplicationInputs_monitoring_case_number_inputs_flags
{
public:
  explicit Init_ApplicationInputs_monitoring_case_number_inputs_flags(::sick_safetyscanners2_interfaces::msg::ApplicationInputs & msg)
  : msg_(msg)
  {}
  Init_ApplicationInputs_linear_velocity_inputs_velocity_0 monitoring_case_number_inputs_flags(::sick_safetyscanners2_interfaces::msg::ApplicationInputs::_monitoring_case_number_inputs_flags_type arg)
  {
    msg_.monitoring_case_number_inputs_flags = std::move(arg);
    return Init_ApplicationInputs_linear_velocity_inputs_velocity_0(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationInputs msg_;
};

class Init_ApplicationInputs_monitoring_case_number_inputs
{
public:
  explicit Init_ApplicationInputs_monitoring_case_number_inputs(::sick_safetyscanners2_interfaces::msg::ApplicationInputs & msg)
  : msg_(msg)
  {}
  Init_ApplicationInputs_monitoring_case_number_inputs_flags monitoring_case_number_inputs(::sick_safetyscanners2_interfaces::msg::ApplicationInputs::_monitoring_case_number_inputs_type arg)
  {
    msg_.monitoring_case_number_inputs = std::move(arg);
    return Init_ApplicationInputs_monitoring_case_number_inputs_flags(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationInputs msg_;
};

class Init_ApplicationInputs_unsafe_inputs_flags
{
public:
  explicit Init_ApplicationInputs_unsafe_inputs_flags(::sick_safetyscanners2_interfaces::msg::ApplicationInputs & msg)
  : msg_(msg)
  {}
  Init_ApplicationInputs_monitoring_case_number_inputs unsafe_inputs_flags(::sick_safetyscanners2_interfaces::msg::ApplicationInputs::_unsafe_inputs_flags_type arg)
  {
    msg_.unsafe_inputs_flags = std::move(arg);
    return Init_ApplicationInputs_monitoring_case_number_inputs(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationInputs msg_;
};

class Init_ApplicationInputs_unsafe_inputs_input_sources
{
public:
  Init_ApplicationInputs_unsafe_inputs_input_sources()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ApplicationInputs_unsafe_inputs_flags unsafe_inputs_input_sources(::sick_safetyscanners2_interfaces::msg::ApplicationInputs::_unsafe_inputs_input_sources_type arg)
  {
    msg_.unsafe_inputs_input_sources = std::move(arg);
    return Init_ApplicationInputs_unsafe_inputs_flags(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationInputs msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sick_safetyscanners2_interfaces::msg::ApplicationInputs>()
{
  return sick_safetyscanners2_interfaces::msg::builder::Init_ApplicationInputs_unsafe_inputs_input_sources();
}

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_INPUTS__BUILDER_HPP_
