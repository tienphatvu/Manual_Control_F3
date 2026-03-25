// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/OutputPaths.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__OUTPUT_PATHS__BUILDER_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__OUTPUT_PATHS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sick_safetyscanners2_interfaces/msg/detail/output_paths__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sick_safetyscanners2_interfaces
{

namespace msg
{

namespace builder
{

class Init_OutputPaths_active_monitoring_case
{
public:
  explicit Init_OutputPaths_active_monitoring_case(::sick_safetyscanners2_interfaces::msg::OutputPaths & msg)
  : msg_(msg)
  {}
  ::sick_safetyscanners2_interfaces::msg::OutputPaths active_monitoring_case(::sick_safetyscanners2_interfaces::msg::OutputPaths::_active_monitoring_case_type arg)
  {
    msg_.active_monitoring_case = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::OutputPaths msg_;
};

class Init_OutputPaths_is_valid
{
public:
  explicit Init_OutputPaths_is_valid(::sick_safetyscanners2_interfaces::msg::OutputPaths & msg)
  : msg_(msg)
  {}
  Init_OutputPaths_active_monitoring_case is_valid(::sick_safetyscanners2_interfaces::msg::OutputPaths::_is_valid_type arg)
  {
    msg_.is_valid = std::move(arg);
    return Init_OutputPaths_active_monitoring_case(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::OutputPaths msg_;
};

class Init_OutputPaths_is_safe
{
public:
  explicit Init_OutputPaths_is_safe(::sick_safetyscanners2_interfaces::msg::OutputPaths & msg)
  : msg_(msg)
  {}
  Init_OutputPaths_is_valid is_safe(::sick_safetyscanners2_interfaces::msg::OutputPaths::_is_safe_type arg)
  {
    msg_.is_safe = std::move(arg);
    return Init_OutputPaths_is_valid(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::OutputPaths msg_;
};

class Init_OutputPaths_status
{
public:
  Init_OutputPaths_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OutputPaths_is_safe status(::sick_safetyscanners2_interfaces::msg::OutputPaths::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_OutputPaths_is_safe(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::OutputPaths msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sick_safetyscanners2_interfaces::msg::OutputPaths>()
{
  return sick_safetyscanners2_interfaces::msg::builder::Init_OutputPaths_status();
}

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__OUTPUT_PATHS__BUILDER_HPP_
