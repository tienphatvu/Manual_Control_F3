// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/Field.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__FIELD__BUILDER_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__FIELD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sick_safetyscanners2_interfaces/msg/detail/field__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sick_safetyscanners2_interfaces
{

namespace msg
{

namespace builder
{

class Init_Field_protective_field
{
public:
  explicit Init_Field_protective_field(::sick_safetyscanners2_interfaces::msg::Field & msg)
  : msg_(msg)
  {}
  ::sick_safetyscanners2_interfaces::msg::Field protective_field(::sick_safetyscanners2_interfaces::msg::Field::_protective_field_type arg)
  {
    msg_.protective_field = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::Field msg_;
};

class Init_Field_angular_resolution
{
public:
  explicit Init_Field_angular_resolution(::sick_safetyscanners2_interfaces::msg::Field & msg)
  : msg_(msg)
  {}
  Init_Field_protective_field angular_resolution(::sick_safetyscanners2_interfaces::msg::Field::_angular_resolution_type arg)
  {
    msg_.angular_resolution = std::move(arg);
    return Init_Field_protective_field(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::Field msg_;
};

class Init_Field_start_angle
{
public:
  explicit Init_Field_start_angle(::sick_safetyscanners2_interfaces::msg::Field & msg)
  : msg_(msg)
  {}
  Init_Field_angular_resolution start_angle(::sick_safetyscanners2_interfaces::msg::Field::_start_angle_type arg)
  {
    msg_.start_angle = std::move(arg);
    return Init_Field_angular_resolution(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::Field msg_;
};

class Init_Field_ranges
{
public:
  Init_Field_ranges()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Field_start_angle ranges(::sick_safetyscanners2_interfaces::msg::Field::_ranges_type arg)
  {
    msg_.ranges = std::move(arg);
    return Init_Field_start_angle(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::Field msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sick_safetyscanners2_interfaces::msg::Field>()
{
  return sick_safetyscanners2_interfaces::msg::builder::Init_Field_ranges();
}

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__FIELD__BUILDER_HPP_
