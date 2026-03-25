// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/MeasurementData.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__MEASUREMENT_DATA__BUILDER_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__MEASUREMENT_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sick_safetyscanners2_interfaces/msg/detail/measurement_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sick_safetyscanners2_interfaces
{

namespace msg
{

namespace builder
{

class Init_MeasurementData_scan_points
{
public:
  explicit Init_MeasurementData_scan_points(::sick_safetyscanners2_interfaces::msg::MeasurementData & msg)
  : msg_(msg)
  {}
  ::sick_safetyscanners2_interfaces::msg::MeasurementData scan_points(::sick_safetyscanners2_interfaces::msg::MeasurementData::_scan_points_type arg)
  {
    msg_.scan_points = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::MeasurementData msg_;
};

class Init_MeasurementData_number_of_beams
{
public:
  Init_MeasurementData_number_of_beams()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MeasurementData_scan_points number_of_beams(::sick_safetyscanners2_interfaces::msg::MeasurementData::_number_of_beams_type arg)
  {
    msg_.number_of_beams = std::move(arg);
    return Init_MeasurementData_scan_points(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::MeasurementData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sick_safetyscanners2_interfaces::msg::MeasurementData>()
{
  return sick_safetyscanners2_interfaces::msg::builder::Init_MeasurementData_number_of_beams();
}

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__MEASUREMENT_DATA__BUILDER_HPP_
