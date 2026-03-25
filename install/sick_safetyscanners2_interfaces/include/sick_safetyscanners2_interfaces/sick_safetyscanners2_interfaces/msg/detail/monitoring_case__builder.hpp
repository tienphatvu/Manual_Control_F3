// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/MonitoringCase.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__MONITORING_CASE__BUILDER_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__MONITORING_CASE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sick_safetyscanners2_interfaces/msg/detail/monitoring_case__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sick_safetyscanners2_interfaces
{

namespace msg
{

namespace builder
{

class Init_MonitoringCase_fields_valid
{
public:
  explicit Init_MonitoringCase_fields_valid(::sick_safetyscanners2_interfaces::msg::MonitoringCase & msg)
  : msg_(msg)
  {}
  ::sick_safetyscanners2_interfaces::msg::MonitoringCase fields_valid(::sick_safetyscanners2_interfaces::msg::MonitoringCase::_fields_valid_type arg)
  {
    msg_.fields_valid = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::MonitoringCase msg_;
};

class Init_MonitoringCase_fields
{
public:
  explicit Init_MonitoringCase_fields(::sick_safetyscanners2_interfaces::msg::MonitoringCase & msg)
  : msg_(msg)
  {}
  Init_MonitoringCase_fields_valid fields(::sick_safetyscanners2_interfaces::msg::MonitoringCase::_fields_type arg)
  {
    msg_.fields = std::move(arg);
    return Init_MonitoringCase_fields_valid(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::MonitoringCase msg_;
};

class Init_MonitoringCase_monitoring_case_number
{
public:
  Init_MonitoringCase_monitoring_case_number()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MonitoringCase_fields monitoring_case_number(::sick_safetyscanners2_interfaces::msg::MonitoringCase::_monitoring_case_number_type arg)
  {
    msg_.monitoring_case_number = std::move(arg);
    return Init_MonitoringCase_fields(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::MonitoringCase msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sick_safetyscanners2_interfaces::msg::MonitoringCase>()
{
  return sick_safetyscanners2_interfaces::msg::builder::Init_MonitoringCase_monitoring_case_number();
}

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__MONITORING_CASE__BUILDER_HPP_
