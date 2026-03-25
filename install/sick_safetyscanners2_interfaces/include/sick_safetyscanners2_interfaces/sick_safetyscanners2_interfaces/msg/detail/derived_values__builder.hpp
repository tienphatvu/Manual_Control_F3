// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/DerivedValues.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__DERIVED_VALUES__BUILDER_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__DERIVED_VALUES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sick_safetyscanners2_interfaces/msg/detail/derived_values__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sick_safetyscanners2_interfaces
{

namespace msg
{

namespace builder
{

class Init_DerivedValues_interbeam_period
{
public:
  explicit Init_DerivedValues_interbeam_period(::sick_safetyscanners2_interfaces::msg::DerivedValues & msg)
  : msg_(msg)
  {}
  ::sick_safetyscanners2_interfaces::msg::DerivedValues interbeam_period(::sick_safetyscanners2_interfaces::msg::DerivedValues::_interbeam_period_type arg)
  {
    msg_.interbeam_period = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::DerivedValues msg_;
};

class Init_DerivedValues_angular_beam_resolution
{
public:
  explicit Init_DerivedValues_angular_beam_resolution(::sick_safetyscanners2_interfaces::msg::DerivedValues & msg)
  : msg_(msg)
  {}
  Init_DerivedValues_interbeam_period angular_beam_resolution(::sick_safetyscanners2_interfaces::msg::DerivedValues::_angular_beam_resolution_type arg)
  {
    msg_.angular_beam_resolution = std::move(arg);
    return Init_DerivedValues_interbeam_period(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::DerivedValues msg_;
};

class Init_DerivedValues_start_angle
{
public:
  explicit Init_DerivedValues_start_angle(::sick_safetyscanners2_interfaces::msg::DerivedValues & msg)
  : msg_(msg)
  {}
  Init_DerivedValues_angular_beam_resolution start_angle(::sick_safetyscanners2_interfaces::msg::DerivedValues::_start_angle_type arg)
  {
    msg_.start_angle = std::move(arg);
    return Init_DerivedValues_angular_beam_resolution(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::DerivedValues msg_;
};

class Init_DerivedValues_scan_time
{
public:
  explicit Init_DerivedValues_scan_time(::sick_safetyscanners2_interfaces::msg::DerivedValues & msg)
  : msg_(msg)
  {}
  Init_DerivedValues_start_angle scan_time(::sick_safetyscanners2_interfaces::msg::DerivedValues::_scan_time_type arg)
  {
    msg_.scan_time = std::move(arg);
    return Init_DerivedValues_start_angle(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::DerivedValues msg_;
};

class Init_DerivedValues_number_of_beams
{
public:
  explicit Init_DerivedValues_number_of_beams(::sick_safetyscanners2_interfaces::msg::DerivedValues & msg)
  : msg_(msg)
  {}
  Init_DerivedValues_scan_time number_of_beams(::sick_safetyscanners2_interfaces::msg::DerivedValues::_number_of_beams_type arg)
  {
    msg_.number_of_beams = std::move(arg);
    return Init_DerivedValues_scan_time(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::DerivedValues msg_;
};

class Init_DerivedValues_multiplication_factor
{
public:
  Init_DerivedValues_multiplication_factor()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DerivedValues_number_of_beams multiplication_factor(::sick_safetyscanners2_interfaces::msg::DerivedValues::_multiplication_factor_type arg)
  {
    msg_.multiplication_factor = std::move(arg);
    return Init_DerivedValues_number_of_beams(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::DerivedValues msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sick_safetyscanners2_interfaces::msg::DerivedValues>()
{
  return sick_safetyscanners2_interfaces::msg::builder::Init_DerivedValues_multiplication_factor();
}

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__DERIVED_VALUES__BUILDER_HPP_
