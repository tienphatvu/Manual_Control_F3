// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/ScanPoint.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__SCAN_POINT__BUILDER_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__SCAN_POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sick_safetyscanners2_interfaces/msg/detail/scan_point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sick_safetyscanners2_interfaces
{

namespace msg
{

namespace builder
{

class Init_ScanPoint_contamination_warning
{
public:
  explicit Init_ScanPoint_contamination_warning(::sick_safetyscanners2_interfaces::msg::ScanPoint & msg)
  : msg_(msg)
  {}
  ::sick_safetyscanners2_interfaces::msg::ScanPoint contamination_warning(::sick_safetyscanners2_interfaces::msg::ScanPoint::_contamination_warning_type arg)
  {
    msg_.contamination_warning = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ScanPoint msg_;
};

class Init_ScanPoint_contamination
{
public:
  explicit Init_ScanPoint_contamination(::sick_safetyscanners2_interfaces::msg::ScanPoint & msg)
  : msg_(msg)
  {}
  Init_ScanPoint_contamination_warning contamination(::sick_safetyscanners2_interfaces::msg::ScanPoint::_contamination_type arg)
  {
    msg_.contamination = std::move(arg);
    return Init_ScanPoint_contamination_warning(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ScanPoint msg_;
};

class Init_ScanPoint_reflector
{
public:
  explicit Init_ScanPoint_reflector(::sick_safetyscanners2_interfaces::msg::ScanPoint & msg)
  : msg_(msg)
  {}
  Init_ScanPoint_contamination reflector(::sick_safetyscanners2_interfaces::msg::ScanPoint::_reflector_type arg)
  {
    msg_.reflector = std::move(arg);
    return Init_ScanPoint_contamination(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ScanPoint msg_;
};

class Init_ScanPoint_glare
{
public:
  explicit Init_ScanPoint_glare(::sick_safetyscanners2_interfaces::msg::ScanPoint & msg)
  : msg_(msg)
  {}
  Init_ScanPoint_reflector glare(::sick_safetyscanners2_interfaces::msg::ScanPoint::_glare_type arg)
  {
    msg_.glare = std::move(arg);
    return Init_ScanPoint_reflector(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ScanPoint msg_;
};

class Init_ScanPoint_infinite
{
public:
  explicit Init_ScanPoint_infinite(::sick_safetyscanners2_interfaces::msg::ScanPoint & msg)
  : msg_(msg)
  {}
  Init_ScanPoint_glare infinite(::sick_safetyscanners2_interfaces::msg::ScanPoint::_infinite_type arg)
  {
    msg_.infinite = std::move(arg);
    return Init_ScanPoint_glare(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ScanPoint msg_;
};

class Init_ScanPoint_valid
{
public:
  explicit Init_ScanPoint_valid(::sick_safetyscanners2_interfaces::msg::ScanPoint & msg)
  : msg_(msg)
  {}
  Init_ScanPoint_infinite valid(::sick_safetyscanners2_interfaces::msg::ScanPoint::_valid_type arg)
  {
    msg_.valid = std::move(arg);
    return Init_ScanPoint_infinite(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ScanPoint msg_;
};

class Init_ScanPoint_reflectivity
{
public:
  explicit Init_ScanPoint_reflectivity(::sick_safetyscanners2_interfaces::msg::ScanPoint & msg)
  : msg_(msg)
  {}
  Init_ScanPoint_valid reflectivity(::sick_safetyscanners2_interfaces::msg::ScanPoint::_reflectivity_type arg)
  {
    msg_.reflectivity = std::move(arg);
    return Init_ScanPoint_valid(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ScanPoint msg_;
};

class Init_ScanPoint_distance
{
public:
  explicit Init_ScanPoint_distance(::sick_safetyscanners2_interfaces::msg::ScanPoint & msg)
  : msg_(msg)
  {}
  Init_ScanPoint_reflectivity distance(::sick_safetyscanners2_interfaces::msg::ScanPoint::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return Init_ScanPoint_reflectivity(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ScanPoint msg_;
};

class Init_ScanPoint_angle
{
public:
  Init_ScanPoint_angle()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ScanPoint_distance angle(::sick_safetyscanners2_interfaces::msg::ScanPoint::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return Init_ScanPoint_distance(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ScanPoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sick_safetyscanners2_interfaces::msg::ScanPoint>()
{
  return sick_safetyscanners2_interfaces::msg::builder::Init_ScanPoint_angle();
}

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__SCAN_POINT__BUILDER_HPP_
