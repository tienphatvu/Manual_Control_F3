// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/ApplicationData.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_DATA__BUILDER_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sick_safetyscanners2_interfaces/msg/detail/application_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sick_safetyscanners2_interfaces
{

namespace msg
{

namespace builder
{

class Init_ApplicationData_outputs
{
public:
  explicit Init_ApplicationData_outputs(::sick_safetyscanners2_interfaces::msg::ApplicationData & msg)
  : msg_(msg)
  {}
  ::sick_safetyscanners2_interfaces::msg::ApplicationData outputs(::sick_safetyscanners2_interfaces::msg::ApplicationData::_outputs_type arg)
  {
    msg_.outputs = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationData msg_;
};

class Init_ApplicationData_inputs
{
public:
  Init_ApplicationData_inputs()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ApplicationData_outputs inputs(::sick_safetyscanners2_interfaces::msg::ApplicationData::_inputs_type arg)
  {
    msg_.inputs = std::move(arg);
    return Init_ApplicationData_outputs(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ApplicationData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sick_safetyscanners2_interfaces::msg::ApplicationData>()
{
  return sick_safetyscanners2_interfaces::msg::builder::Init_ApplicationData_inputs();
}

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_DATA__BUILDER_HPP_
