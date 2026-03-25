// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sick_safetyscanners2_interfaces:srv/FieldData.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__SRV__DETAIL__FIELD_DATA__BUILDER_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__SRV__DETAIL__FIELD_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sick_safetyscanners2_interfaces/srv/detail/field_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sick_safetyscanners2_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::sick_safetyscanners2_interfaces::srv::FieldData_Request>()
{
  return ::sick_safetyscanners2_interfaces::srv::FieldData_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace sick_safetyscanners2_interfaces


namespace sick_safetyscanners2_interfaces
{

namespace srv
{

namespace builder
{

class Init_FieldData_Response_monitoring_cases
{
public:
  explicit Init_FieldData_Response_monitoring_cases(::sick_safetyscanners2_interfaces::srv::FieldData_Response & msg)
  : msg_(msg)
  {}
  ::sick_safetyscanners2_interfaces::srv::FieldData_Response monitoring_cases(::sick_safetyscanners2_interfaces::srv::FieldData_Response::_monitoring_cases_type arg)
  {
    msg_.monitoring_cases = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::srv::FieldData_Response msg_;
};

class Init_FieldData_Response_device_name
{
public:
  explicit Init_FieldData_Response_device_name(::sick_safetyscanners2_interfaces::srv::FieldData_Response & msg)
  : msg_(msg)
  {}
  Init_FieldData_Response_monitoring_cases device_name(::sick_safetyscanners2_interfaces::srv::FieldData_Response::_device_name_type arg)
  {
    msg_.device_name = std::move(arg);
    return Init_FieldData_Response_monitoring_cases(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::srv::FieldData_Response msg_;
};

class Init_FieldData_Response_fields
{
public:
  Init_FieldData_Response_fields()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FieldData_Response_device_name fields(::sick_safetyscanners2_interfaces::srv::FieldData_Response::_fields_type arg)
  {
    msg_.fields = std::move(arg);
    return Init_FieldData_Response_device_name(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::srv::FieldData_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::sick_safetyscanners2_interfaces::srv::FieldData_Response>()
{
  return sick_safetyscanners2_interfaces::srv::builder::Init_FieldData_Response_fields();
}

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__SRV__DETAIL__FIELD_DATA__BUILDER_HPP_
