// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/DataHeader.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__DATA_HEADER__BUILDER_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__DATA_HEADER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sick_safetyscanners2_interfaces/msg/detail/data_header__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sick_safetyscanners2_interfaces
{

namespace msg
{

namespace builder
{

class Init_DataHeader_timestamp_time
{
public:
  explicit Init_DataHeader_timestamp_time(::sick_safetyscanners2_interfaces::msg::DataHeader & msg)
  : msg_(msg)
  {}
  ::sick_safetyscanners2_interfaces::msg::DataHeader timestamp_time(::sick_safetyscanners2_interfaces::msg::DataHeader::_timestamp_time_type arg)
  {
    msg_.timestamp_time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::DataHeader msg_;
};

class Init_DataHeader_timestamp_date
{
public:
  explicit Init_DataHeader_timestamp_date(::sick_safetyscanners2_interfaces::msg::DataHeader & msg)
  : msg_(msg)
  {}
  Init_DataHeader_timestamp_time timestamp_date(::sick_safetyscanners2_interfaces::msg::DataHeader::_timestamp_date_type arg)
  {
    msg_.timestamp_date = std::move(arg);
    return Init_DataHeader_timestamp_time(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::DataHeader msg_;
};

class Init_DataHeader_scan_number
{
public:
  explicit Init_DataHeader_scan_number(::sick_safetyscanners2_interfaces::msg::DataHeader & msg)
  : msg_(msg)
  {}
  Init_DataHeader_timestamp_date scan_number(::sick_safetyscanners2_interfaces::msg::DataHeader::_scan_number_type arg)
  {
    msg_.scan_number = std::move(arg);
    return Init_DataHeader_timestamp_date(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::DataHeader msg_;
};

class Init_DataHeader_sequence_number
{
public:
  explicit Init_DataHeader_sequence_number(::sick_safetyscanners2_interfaces::msg::DataHeader & msg)
  : msg_(msg)
  {}
  Init_DataHeader_scan_number sequence_number(::sick_safetyscanners2_interfaces::msg::DataHeader::_sequence_number_type arg)
  {
    msg_.sequence_number = std::move(arg);
    return Init_DataHeader_scan_number(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::DataHeader msg_;
};

class Init_DataHeader_channel_number
{
public:
  explicit Init_DataHeader_channel_number(::sick_safetyscanners2_interfaces::msg::DataHeader & msg)
  : msg_(msg)
  {}
  Init_DataHeader_sequence_number channel_number(::sick_safetyscanners2_interfaces::msg::DataHeader::_channel_number_type arg)
  {
    msg_.channel_number = std::move(arg);
    return Init_DataHeader_sequence_number(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::DataHeader msg_;
};

class Init_DataHeader_serial_number_of_channel_plug
{
public:
  explicit Init_DataHeader_serial_number_of_channel_plug(::sick_safetyscanners2_interfaces::msg::DataHeader & msg)
  : msg_(msg)
  {}
  Init_DataHeader_channel_number serial_number_of_channel_plug(::sick_safetyscanners2_interfaces::msg::DataHeader::_serial_number_of_channel_plug_type arg)
  {
    msg_.serial_number_of_channel_plug = std::move(arg);
    return Init_DataHeader_channel_number(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::DataHeader msg_;
};

class Init_DataHeader_serial_number_of_device
{
public:
  explicit Init_DataHeader_serial_number_of_device(::sick_safetyscanners2_interfaces::msg::DataHeader & msg)
  : msg_(msg)
  {}
  Init_DataHeader_serial_number_of_channel_plug serial_number_of_device(::sick_safetyscanners2_interfaces::msg::DataHeader::_serial_number_of_device_type arg)
  {
    msg_.serial_number_of_device = std::move(arg);
    return Init_DataHeader_serial_number_of_channel_plug(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::DataHeader msg_;
};

class Init_DataHeader_version_release
{
public:
  explicit Init_DataHeader_version_release(::sick_safetyscanners2_interfaces::msg::DataHeader & msg)
  : msg_(msg)
  {}
  Init_DataHeader_serial_number_of_device version_release(::sick_safetyscanners2_interfaces::msg::DataHeader::_version_release_type arg)
  {
    msg_.version_release = std::move(arg);
    return Init_DataHeader_serial_number_of_device(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::DataHeader msg_;
};

class Init_DataHeader_version_minor_version
{
public:
  explicit Init_DataHeader_version_minor_version(::sick_safetyscanners2_interfaces::msg::DataHeader & msg)
  : msg_(msg)
  {}
  Init_DataHeader_version_release version_minor_version(::sick_safetyscanners2_interfaces::msg::DataHeader::_version_minor_version_type arg)
  {
    msg_.version_minor_version = std::move(arg);
    return Init_DataHeader_version_release(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::DataHeader msg_;
};

class Init_DataHeader_version_major_version
{
public:
  explicit Init_DataHeader_version_major_version(::sick_safetyscanners2_interfaces::msg::DataHeader & msg)
  : msg_(msg)
  {}
  Init_DataHeader_version_minor_version version_major_version(::sick_safetyscanners2_interfaces::msg::DataHeader::_version_major_version_type arg)
  {
    msg_.version_major_version = std::move(arg);
    return Init_DataHeader_version_minor_version(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::DataHeader msg_;
};

class Init_DataHeader_version_version
{
public:
  Init_DataHeader_version_version()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DataHeader_version_major_version version_version(::sick_safetyscanners2_interfaces::msg::DataHeader::_version_version_type arg)
  {
    msg_.version_version = std::move(arg);
    return Init_DataHeader_version_major_version(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::msg::DataHeader msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sick_safetyscanners2_interfaces::msg::DataHeader>()
{
  return sick_safetyscanners2_interfaces::msg::builder::Init_DataHeader_version_version();
}

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__DATA_HEADER__BUILDER_HPP_
