// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/RawMicroScanData.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__RAW_MICRO_SCAN_DATA__BUILDER_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__RAW_MICRO_SCAN_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sick_safetyscanners2_interfaces/msg/detail/raw_micro_scan_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sick_safetyscanners2_interfaces
{

namespace msg
{

namespace builder
{

class Init_RawMicroScanData_application_data
{
public:
  explicit Init_RawMicroScanData_application_data(::sick_safetyscanners2_interfaces::msg::RawMicroScanData & msg)
  : msg_(msg)
  {}
  ::sick_safetyscanners2_interfaces::msg::RawMicroScanData application_data(::sick_safetyscanners2_interfaces::msg::RawMicroScanData::_application_data_type arg)
  {
    msg_.application_data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::RawMicroScanData msg_;
};

class Init_RawMicroScanData_intrusion_data
{
public:
  explicit Init_RawMicroScanData_intrusion_data(::sick_safetyscanners2_interfaces::msg::RawMicroScanData & msg)
  : msg_(msg)
  {}
  Init_RawMicroScanData_application_data intrusion_data(::sick_safetyscanners2_interfaces::msg::RawMicroScanData::_intrusion_data_type arg)
  {
    msg_.intrusion_data = std::move(arg);
    return Init_RawMicroScanData_application_data(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::RawMicroScanData msg_;
};

class Init_RawMicroScanData_measurement_data
{
public:
  explicit Init_RawMicroScanData_measurement_data(::sick_safetyscanners2_interfaces::msg::RawMicroScanData & msg)
  : msg_(msg)
  {}
  Init_RawMicroScanData_intrusion_data measurement_data(::sick_safetyscanners2_interfaces::msg::RawMicroScanData::_measurement_data_type arg)
  {
    msg_.measurement_data = std::move(arg);
    return Init_RawMicroScanData_intrusion_data(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::RawMicroScanData msg_;
};

class Init_RawMicroScanData_general_system_state
{
public:
  explicit Init_RawMicroScanData_general_system_state(::sick_safetyscanners2_interfaces::msg::RawMicroScanData & msg)
  : msg_(msg)
  {}
  Init_RawMicroScanData_measurement_data general_system_state(::sick_safetyscanners2_interfaces::msg::RawMicroScanData::_general_system_state_type arg)
  {
    msg_.general_system_state = std::move(arg);
    return Init_RawMicroScanData_measurement_data(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::RawMicroScanData msg_;
};

class Init_RawMicroScanData_derived_values
{
public:
  explicit Init_RawMicroScanData_derived_values(::sick_safetyscanners2_interfaces::msg::RawMicroScanData & msg)
  : msg_(msg)
  {}
  Init_RawMicroScanData_general_system_state derived_values(::sick_safetyscanners2_interfaces::msg::RawMicroScanData::_derived_values_type arg)
  {
    msg_.derived_values = std::move(arg);
    return Init_RawMicroScanData_general_system_state(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::RawMicroScanData msg_;
};

class Init_RawMicroScanData_header
{
public:
  Init_RawMicroScanData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RawMicroScanData_derived_values header(::sick_safetyscanners2_interfaces::msg::RawMicroScanData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RawMicroScanData_derived_values(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::RawMicroScanData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sick_safetyscanners2_interfaces::msg::RawMicroScanData>()
{
  return sick_safetyscanners2_interfaces::msg::builder::Init_RawMicroScanData_header();
}

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__RAW_MICRO_SCAN_DATA__BUILDER_HPP_
