// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sick_safetyscanners2_interfaces:srv/StatusOverview.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__SRV__DETAIL__STATUS_OVERVIEW__STRUCT_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__SRV__DETAIL__STATUS_OVERVIEW__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__sick_safetyscanners2_interfaces__srv__StatusOverview_Request __attribute__((deprecated))
#else
# define DEPRECATED__sick_safetyscanners2_interfaces__srv__StatusOverview_Request __declspec(deprecated)
#endif

namespace sick_safetyscanners2_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct StatusOverview_Request_
{
  using Type = StatusOverview_Request_<ContainerAllocator>;

  explicit StatusOverview_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit StatusOverview_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    sick_safetyscanners2_interfaces::srv::StatusOverview_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const sick_safetyscanners2_interfaces::srv::StatusOverview_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::srv::StatusOverview_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::srv::StatusOverview_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::srv::StatusOverview_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::srv::StatusOverview_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::srv::StatusOverview_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::srv::StatusOverview_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::srv::StatusOverview_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::srv::StatusOverview_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sick_safetyscanners2_interfaces__srv__StatusOverview_Request
    std::shared_ptr<sick_safetyscanners2_interfaces::srv::StatusOverview_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sick_safetyscanners2_interfaces__srv__StatusOverview_Request
    std::shared_ptr<sick_safetyscanners2_interfaces::srv::StatusOverview_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StatusOverview_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const StatusOverview_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StatusOverview_Request_

// alias to use template instance with default allocator
using StatusOverview_Request =
  sick_safetyscanners2_interfaces::srv::StatusOverview_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace sick_safetyscanners2_interfaces


#ifndef _WIN32
# define DEPRECATED__sick_safetyscanners2_interfaces__srv__StatusOverview_Response __attribute__((deprecated))
#else
# define DEPRECATED__sick_safetyscanners2_interfaces__srv__StatusOverview_Response __declspec(deprecated)
#endif

namespace sick_safetyscanners2_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct StatusOverview_Response_
{
  using Type = StatusOverview_Response_<ContainerAllocator>;

  explicit StatusOverview_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->version_c_version = "";
      this->version_major_version_number = 0;
      this->version_minor_version_number = 0;
      this->version_release_number = 0;
      this->device_state = 0;
      this->config_state = 0;
      this->application_state = 0;
      this->current_time_power_on_count = 0ul;
      this->current_time = "";
      this->current_time_time = 0ul;
      this->current_time_date = 0;
      this->error_info_code = 0ul;
      this->error_info_time = "";
      this->error_info_time_time = 0ul;
      this->error_info_time_date = 0;
    }
  }

  explicit StatusOverview_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : version_c_version(_alloc),
    current_time(_alloc),
    error_info_time(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->version_c_version = "";
      this->version_major_version_number = 0;
      this->version_minor_version_number = 0;
      this->version_release_number = 0;
      this->device_state = 0;
      this->config_state = 0;
      this->application_state = 0;
      this->current_time_power_on_count = 0ul;
      this->current_time = "";
      this->current_time_time = 0ul;
      this->current_time_date = 0;
      this->error_info_code = 0ul;
      this->error_info_time = "";
      this->error_info_time_time = 0ul;
      this->error_info_time_date = 0;
    }
  }

  // field types and members
  using _version_c_version_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _version_c_version_type version_c_version;
  using _version_major_version_number_type =
    uint8_t;
  _version_major_version_number_type version_major_version_number;
  using _version_minor_version_number_type =
    uint8_t;
  _version_minor_version_number_type version_minor_version_number;
  using _version_release_number_type =
    uint8_t;
  _version_release_number_type version_release_number;
  using _device_state_type =
    uint8_t;
  _device_state_type device_state;
  using _config_state_type =
    uint8_t;
  _config_state_type config_state;
  using _application_state_type =
    uint8_t;
  _application_state_type application_state;
  using _current_time_power_on_count_type =
    uint32_t;
  _current_time_power_on_count_type current_time_power_on_count;
  using _current_time_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _current_time_type current_time;
  using _current_time_time_type =
    uint32_t;
  _current_time_time_type current_time_time;
  using _current_time_date_type =
    uint16_t;
  _current_time_date_type current_time_date;
  using _error_info_code_type =
    uint32_t;
  _error_info_code_type error_info_code;
  using _error_info_time_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _error_info_time_type error_info_time;
  using _error_info_time_time_type =
    uint32_t;
  _error_info_time_time_type error_info_time_time;
  using _error_info_time_date_type =
    uint16_t;
  _error_info_time_date_type error_info_time_date;

  // setters for named parameter idiom
  Type & set__version_c_version(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->version_c_version = _arg;
    return *this;
  }
  Type & set__version_major_version_number(
    const uint8_t & _arg)
  {
    this->version_major_version_number = _arg;
    return *this;
  }
  Type & set__version_minor_version_number(
    const uint8_t & _arg)
  {
    this->version_minor_version_number = _arg;
    return *this;
  }
  Type & set__version_release_number(
    const uint8_t & _arg)
  {
    this->version_release_number = _arg;
    return *this;
  }
  Type & set__device_state(
    const uint8_t & _arg)
  {
    this->device_state = _arg;
    return *this;
  }
  Type & set__config_state(
    const uint8_t & _arg)
  {
    this->config_state = _arg;
    return *this;
  }
  Type & set__application_state(
    const uint8_t & _arg)
  {
    this->application_state = _arg;
    return *this;
  }
  Type & set__current_time_power_on_count(
    const uint32_t & _arg)
  {
    this->current_time_power_on_count = _arg;
    return *this;
  }
  Type & set__current_time(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->current_time = _arg;
    return *this;
  }
  Type & set__current_time_time(
    const uint32_t & _arg)
  {
    this->current_time_time = _arg;
    return *this;
  }
  Type & set__current_time_date(
    const uint16_t & _arg)
  {
    this->current_time_date = _arg;
    return *this;
  }
  Type & set__error_info_code(
    const uint32_t & _arg)
  {
    this->error_info_code = _arg;
    return *this;
  }
  Type & set__error_info_time(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->error_info_time = _arg;
    return *this;
  }
  Type & set__error_info_time_time(
    const uint32_t & _arg)
  {
    this->error_info_time_time = _arg;
    return *this;
  }
  Type & set__error_info_time_date(
    const uint16_t & _arg)
  {
    this->error_info_time_date = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sick_safetyscanners2_interfaces::srv::StatusOverview_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const sick_safetyscanners2_interfaces::srv::StatusOverview_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::srv::StatusOverview_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::srv::StatusOverview_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::srv::StatusOverview_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::srv::StatusOverview_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::srv::StatusOverview_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::srv::StatusOverview_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::srv::StatusOverview_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::srv::StatusOverview_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sick_safetyscanners2_interfaces__srv__StatusOverview_Response
    std::shared_ptr<sick_safetyscanners2_interfaces::srv::StatusOverview_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sick_safetyscanners2_interfaces__srv__StatusOverview_Response
    std::shared_ptr<sick_safetyscanners2_interfaces::srv::StatusOverview_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StatusOverview_Response_ & other) const
  {
    if (this->version_c_version != other.version_c_version) {
      return false;
    }
    if (this->version_major_version_number != other.version_major_version_number) {
      return false;
    }
    if (this->version_minor_version_number != other.version_minor_version_number) {
      return false;
    }
    if (this->version_release_number != other.version_release_number) {
      return false;
    }
    if (this->device_state != other.device_state) {
      return false;
    }
    if (this->config_state != other.config_state) {
      return false;
    }
    if (this->application_state != other.application_state) {
      return false;
    }
    if (this->current_time_power_on_count != other.current_time_power_on_count) {
      return false;
    }
    if (this->current_time != other.current_time) {
      return false;
    }
    if (this->current_time_time != other.current_time_time) {
      return false;
    }
    if (this->current_time_date != other.current_time_date) {
      return false;
    }
    if (this->error_info_code != other.error_info_code) {
      return false;
    }
    if (this->error_info_time != other.error_info_time) {
      return false;
    }
    if (this->error_info_time_time != other.error_info_time_time) {
      return false;
    }
    if (this->error_info_time_date != other.error_info_time_date) {
      return false;
    }
    return true;
  }
  bool operator!=(const StatusOverview_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StatusOverview_Response_

// alias to use template instance with default allocator
using StatusOverview_Response =
  sick_safetyscanners2_interfaces::srv::StatusOverview_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace sick_safetyscanners2_interfaces

namespace sick_safetyscanners2_interfaces
{

namespace srv
{

struct StatusOverview
{
  using Request = sick_safetyscanners2_interfaces::srv::StatusOverview_Request;
  using Response = sick_safetyscanners2_interfaces::srv::StatusOverview_Response;
};

}  // namespace srv

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__SRV__DETAIL__STATUS_OVERVIEW__STRUCT_HPP_
