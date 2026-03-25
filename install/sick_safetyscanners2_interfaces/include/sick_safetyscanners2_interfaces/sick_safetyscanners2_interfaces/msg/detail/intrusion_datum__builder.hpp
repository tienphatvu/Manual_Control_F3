// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/IntrusionDatum.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__INTRUSION_DATUM__BUILDER_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__INTRUSION_DATUM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sick_safetyscanners2_interfaces/msg/detail/intrusion_datum__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sick_safetyscanners2_interfaces
{

namespace msg
{

namespace builder
{

class Init_IntrusionDatum_flags
{
public:
  explicit Init_IntrusionDatum_flags(::sick_safetyscanners2_interfaces::msg::IntrusionDatum & msg)
  : msg_(msg)
  {}
  ::sick_safetyscanners2_interfaces::msg::IntrusionDatum flags(::sick_safetyscanners2_interfaces::msg::IntrusionDatum::_flags_type arg)
  {
    msg_.flags = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::IntrusionDatum msg_;
};

class Init_IntrusionDatum_size
{
public:
  Init_IntrusionDatum_size()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IntrusionDatum_flags size(::sick_safetyscanners2_interfaces::msg::IntrusionDatum::_size_type arg)
  {
    msg_.size = std::move(arg);
    return Init_IntrusionDatum_flags(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::IntrusionDatum msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sick_safetyscanners2_interfaces::msg::IntrusionDatum>()
{
  return sick_safetyscanners2_interfaces::msg::builder::Init_IntrusionDatum_size();
}

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__INTRUSION_DATUM__BUILDER_HPP_
