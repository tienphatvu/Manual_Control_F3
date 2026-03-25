// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sick_safetyscanners2_interfaces:srv/StatusOverview.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__SRV__DETAIL__STATUS_OVERVIEW__BUILDER_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__SRV__DETAIL__STATUS_OVERVIEW__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sick_safetyscanners2_interfaces/srv/detail/status_overview__struct.hpp"
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
auto build<::sick_safetyscanners2_interfaces::srv::StatusOverview_Request>()
{
  return ::sick_safetyscanners2_interfaces::srv::StatusOverview_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace sick_safetyscanners2_interfaces


namespace sick_safetyscanners2_interfaces
{

namespace srv
{

namespace builder
{

class Init_StatusOverview_Response_error_info_time_date
{
public:
  explicit Init_StatusOverview_Response_error_info_time_date(::sick_safetyscanners2_interfaces::srv::StatusOverview_Response & msg)
  : msg_(msg)
  {}
  ::sick_safetyscanners2_interfaces::srv::StatusOverview_Response error_info_time_date(::sick_safetyscanners2_interfaces::srv::StatusOverview_Response::_error_info_time_date_type arg)
  {
    msg_.error_info_time_date = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::srv::StatusOverview_Response msg_;
};

class Init_StatusOverview_Response_error_info_time_time
{
public:
  explicit Init_StatusOverview_Response_error_info_time_time(::sick_safetyscanners2_interfaces::srv::StatusOverview_Response & msg)
  : msg_(msg)
  {}
  Init_StatusOverview_Response_error_info_time_date error_info_time_time(::sick_safetyscanners2_interfaces::srv::StatusOverview_Response::_error_info_time_time_type arg)
  {
    msg_.error_info_time_time = std::move(arg);
    return Init_StatusOverview_Response_error_info_time_date(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::srv::StatusOverview_Response msg_;
};

class Init_StatusOverview_Response_error_info_time
{
public:
  explicit Init_StatusOverview_Response_error_info_time(::sick_safetyscanners2_interfaces::srv::StatusOverview_Response & msg)
  : msg_(msg)
  {}
  Init_StatusOverview_Response_error_info_time_time error_info_time(::sick_safetyscanners2_interfaces::srv::StatusOverview_Response::_error_info_time_type arg)
  {
    msg_.error_info_time = std::move(arg);
    return Init_StatusOverview_Response_error_info_time_time(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::srv::StatusOverview_Response msg_;
};

class Init_StatusOverview_Response_error_info_code
{
public:
  explicit Init_StatusOverview_Response_error_info_code(::sick_safetyscanners2_interfaces::srv::StatusOverview_Response & msg)
  : msg_(msg)
  {}
  Init_StatusOverview_Response_error_info_time error_info_code(::sick_safetyscanners2_interfaces::srv::StatusOverview_Response::_error_info_code_type arg)
  {
    msg_.error_info_code = std::move(arg);
    return Init_StatusOverview_Response_error_info_time(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::srv::StatusOverview_Response msg_;
};

class Init_StatusOverview_Response_current_time_date
{
public:
  explicit Init_StatusOverview_Response_current_time_date(::sick_safetyscanners2_interfaces::srv::StatusOverview_Response & msg)
  : msg_(msg)
  {}
  Init_StatusOverview_Response_error_info_code current_time_date(::sick_safetyscanners2_interfaces::srv::StatusOverview_Response::_current_time_date_type arg)
  {
    msg_.current_time_date = std::move(arg);
    return Init_StatusOverview_Response_error_info_code(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::srv::StatusOverview_Response msg_;
};

class Init_StatusOverview_Response_current_time_time
{
public:
  explicit Init_StatusOverview_Response_current_time_time(::sick_safetyscanners2_interfaces::srv::StatusOverview_Response & msg)
  : msg_(msg)
  {}
  Init_StatusOverview_Response_current_time_date current_time_time(::sick_safetyscanners2_interfaces::srv::StatusOverview_Response::_current_time_time_type arg)
  {
    msg_.current_time_time = std::move(arg);
    return Init_StatusOverview_Response_current_time_date(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::srv::StatusOverview_Response msg_;
};

class Init_StatusOverview_Response_current_time
{
public:
  explicit Init_StatusOverview_Response_current_time(::sick_safetyscanners2_interfaces::srv::StatusOverview_Response & msg)
  : msg_(msg)
  {}
  Init_StatusOverview_Response_current_time_time current_time(::sick_safetyscanners2_interfaces::srv::StatusOverview_Response::_current_time_type arg)
  {
    msg_.current_time = std::move(arg);
    return Init_StatusOverview_Response_current_time_time(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::srv::StatusOverview_Response msg_;
};

class Init_StatusOverview_Response_current_time_power_on_count
{
public:
  explicit Init_StatusOverview_Response_current_time_power_on_count(::sick_safetyscanners2_interfaces::srv::StatusOverview_Response & msg)
  : msg_(msg)
  {}
  Init_StatusOverview_Response_current_time current_time_power_on_count(::sick_safetyscanners2_interfaces::srv::StatusOverview_Response::_current_time_power_on_count_type arg)
  {
    msg_.current_time_power_on_count = std::move(arg);
    return Init_StatusOverview_Response_current_time(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::srv::StatusOverview_Response msg_;
};

class Init_StatusOverview_Response_application_state
{
public:
  explicit Init_StatusOverview_Response_application_state(::sick_safetyscanners2_interfaces::srv::StatusOverview_Response & msg)
  : msg_(msg)
  {}
  Init_StatusOverview_Response_current_time_power_on_count application_state(::sick_safetyscanners2_interfaces::srv::StatusOverview_Response::_application_state_type arg)
  {
    msg_.application_state = std::move(arg);
    return Init_StatusOverview_Response_current_time_power_on_count(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::srv::StatusOverview_Response msg_;
};

class Init_StatusOverview_Response_config_state
{
public:
  explicit Init_StatusOverview_Response_config_state(::sick_safetyscanners2_interfaces::srv::StatusOverview_Response & msg)
  : msg_(msg)
  {}
  Init_StatusOverview_Response_application_state config_state(::sick_safetyscanners2_interfaces::srv::StatusOverview_Response::_config_state_type arg)
  {
    msg_.config_state = std::move(arg);
    return Init_StatusOverview_Response_application_state(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::srv::StatusOverview_Response msg_;
};

class Init_StatusOverview_Response_device_state
{
public:
  explicit Init_StatusOverview_Response_device_state(::sick_safetyscanners2_interfaces::srv::StatusOverview_Response & msg)
  : msg_(msg)
  {}
  Init_StatusOverview_Response_config_state device_state(::sick_safetyscanners2_interfaces::srv::StatusOverview_Response::_device_state_type arg)
  {
    msg_.device_state = std::move(arg);
    return Init_StatusOverview_Response_config_state(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::srv::StatusOverview_Response msg_;
};

class Init_StatusOverview_Response_version_release_number
{
public:
  explicit Init_StatusOverview_Response_version_release_number(::sick_safetyscanners2_interfaces::srv::StatusOverview_Response & msg)
  : msg_(msg)
  {}
  Init_StatusOverview_Response_device_state version_release_number(::sick_safetyscanners2_interfaces::srv::StatusOverview_Response::_version_release_number_type arg)
  {
    msg_.version_release_number = std::move(arg);
    return Init_StatusOverview_Response_device_state(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::srv::StatusOverview_Response msg_;
};

class Init_StatusOverview_Response_version_minor_version_number
{
public:
  explicit Init_StatusOverview_Response_version_minor_version_number(::sick_safetyscanners2_interfaces::srv::StatusOverview_Response & msg)
  : msg_(msg)
  {}
  Init_StatusOverview_Response_version_release_number version_minor_version_number(::sick_safetyscanners2_interfaces::srv::StatusOverview_Response::_version_minor_version_number_type arg)
  {
    msg_.version_minor_version_number = std::move(arg);
    return Init_StatusOverview_Response_version_release_number(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::srv::StatusOverview_Response msg_;
};

class Init_StatusOverview_Response_version_major_version_number
{
public:
  explicit Init_StatusOverview_Response_version_major_version_number(::sick_safetyscanners2_interfaces::srv::StatusOverview_Response & msg)
  : msg_(msg)
  {}
  Init_StatusOverview_Response_version_minor_version_number version_major_version_number(::sick_safetyscanners2_interfaces::srv::StatusOverview_Response::_version_major_version_number_type arg)
  {
    msg_.version_major_version_number = std::move(arg);
    return Init_StatusOverview_Response_version_minor_version_number(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::srv::StatusOverview_Response msg_;
};

class Init_StatusOverview_Response_version_c_version
{
public:
  Init_StatusOverview_Response_version_c_version()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StatusOverview_Response_version_major_version_number version_c_version(::sick_safetyscanners2_interfaces::srv::StatusOverview_Response::_version_c_version_type arg)
  {
    msg_.version_c_version = std::move(arg);
    return Init_StatusOverview_Response_version_major_version_number(msg_);
  }

private:
  ::sick_safetyscanners2_interfaces::srv::StatusOverview_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::sick_safetyscanners2_interfaces::srv::StatusOverview_Response>()
{
  return sick_safetyscanners2_interfaces::srv::builder::Init_StatusOverview_Response_version_c_version();
}

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__SRV__DETAIL__STATUS_OVERVIEW__BUILDER_HPP_
