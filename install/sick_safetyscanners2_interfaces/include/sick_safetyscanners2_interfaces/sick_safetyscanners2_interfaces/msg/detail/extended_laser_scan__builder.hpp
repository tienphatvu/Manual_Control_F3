// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/ExtendedLaserScan.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__EXTENDED_LASER_SCAN__BUILDER_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__EXTENDED_LASER_SCAN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sick_safetyscanners2_interfaces/msg/detail/extended_laser_scan__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sick_safetyscanners2_interfaces
{

namespace msg
{

namespace builder
{

class Init_ExtendedLaserScan_intrusion
{
public:
  explicit Init_ExtendedLaserScan_intrusion(::sick_safetyscanners2_interfaces::msg::ExtendedLaserScan & msg)
  : msg_(msg)
  {}
  ::sick_safetyscanners2_interfaces::msg::ExtendedLaserScan intrusion(::sick_safetyscanners2_interfaces::msg::ExtendedLaserScan::_intrusion_type arg)
  {
    msg_.intrusion = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ExtendedLaserScan msg_;
};

class Init_ExtendedLaserScan_reflektor_median
{
public:
  explicit Init_ExtendedLaserScan_reflektor_median(::sick_safetyscanners2_interfaces::msg::ExtendedLaserScan & msg)
  : msg_(msg)
  {}
  Init_ExtendedLaserScan_intrusion reflektor_median(::sick_safetyscanners2_interfaces::msg::ExtendedLaserScan::_reflektor_median_type arg)
  {
    msg_.reflektor_median = std::move(arg);
    return Init_ExtendedLaserScan_intrusion(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ExtendedLaserScan msg_;
};

class Init_ExtendedLaserScan_reflektor_status
{
public:
  explicit Init_ExtendedLaserScan_reflektor_status(::sick_safetyscanners2_interfaces::msg::ExtendedLaserScan & msg)
  : msg_(msg)
  {}
  Init_ExtendedLaserScan_reflektor_median reflektor_status(::sick_safetyscanners2_interfaces::msg::ExtendedLaserScan::_reflektor_status_type arg)
  {
    msg_.reflektor_status = std::move(arg);
    return Init_ExtendedLaserScan_reflektor_median(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ExtendedLaserScan msg_;
};

class Init_ExtendedLaserScan_laser_scan
{
public:
  Init_ExtendedLaserScan_laser_scan()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExtendedLaserScan_reflektor_status laser_scan(::sick_safetyscanners2_interfaces::msg::ExtendedLaserScan::_laser_scan_type arg)
  {
    msg_.laser_scan = std::move(arg);
    return Init_ExtendedLaserScan_reflektor_status(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::ExtendedLaserScan msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sick_safetyscanners2_interfaces::msg::ExtendedLaserScan>()
{
  return sick_safetyscanners2_interfaces::msg::builder::Init_ExtendedLaserScan_laser_scan();
}

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__EXTENDED_LASER_SCAN__BUILDER_HPP_
