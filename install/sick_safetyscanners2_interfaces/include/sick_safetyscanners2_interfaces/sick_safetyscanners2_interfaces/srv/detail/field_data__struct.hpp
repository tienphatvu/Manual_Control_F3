// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sick_safetyscanners2_interfaces:srv/FieldData.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__SRV__DETAIL__FIELD_DATA__STRUCT_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__SRV__DETAIL__FIELD_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__sick_safetyscanners2_interfaces__srv__FieldData_Request __attribute__((deprecated))
#else
# define DEPRECATED__sick_safetyscanners2_interfaces__srv__FieldData_Request __declspec(deprecated)
#endif

namespace sick_safetyscanners2_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct FieldData_Request_
{
  using Type = FieldData_Request_<ContainerAllocator>;

  explicit FieldData_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit FieldData_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    sick_safetyscanners2_interfaces::srv::FieldData_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const sick_safetyscanners2_interfaces::srv::FieldData_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::srv::FieldData_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::srv::FieldData_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::srv::FieldData_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::srv::FieldData_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::srv::FieldData_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::srv::FieldData_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::srv::FieldData_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::srv::FieldData_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sick_safetyscanners2_interfaces__srv__FieldData_Request
    std::shared_ptr<sick_safetyscanners2_interfaces::srv::FieldData_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sick_safetyscanners2_interfaces__srv__FieldData_Request
    std::shared_ptr<sick_safetyscanners2_interfaces::srv::FieldData_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FieldData_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const FieldData_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FieldData_Request_

// alias to use template instance with default allocator
using FieldData_Request =
  sick_safetyscanners2_interfaces::srv::FieldData_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace sick_safetyscanners2_interfaces


// Include directives for member types
// Member 'fields'
#include "sick_safetyscanners2_interfaces/msg/detail/field__struct.hpp"
// Member 'monitoring_cases'
#include "sick_safetyscanners2_interfaces/msg/detail/monitoring_case__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__sick_safetyscanners2_interfaces__srv__FieldData_Response __attribute__((deprecated))
#else
# define DEPRECATED__sick_safetyscanners2_interfaces__srv__FieldData_Response __declspec(deprecated)
#endif

namespace sick_safetyscanners2_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct FieldData_Response_
{
  using Type = FieldData_Response_<ContainerAllocator>;

  explicit FieldData_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->device_name = "";
    }
  }

  explicit FieldData_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : device_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->device_name = "";
    }
  }

  // field types and members
  using _fields_type =
    std::vector<sick_safetyscanners2_interfaces::msg::Field_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sick_safetyscanners2_interfaces::msg::Field_<ContainerAllocator>>>;
  _fields_type fields;
  using _device_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _device_name_type device_name;
  using _monitoring_cases_type =
    std::vector<sick_safetyscanners2_interfaces::msg::MonitoringCase_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sick_safetyscanners2_interfaces::msg::MonitoringCase_<ContainerAllocator>>>;
  _monitoring_cases_type monitoring_cases;

  // setters for named parameter idiom
  Type & set__fields(
    const std::vector<sick_safetyscanners2_interfaces::msg::Field_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sick_safetyscanners2_interfaces::msg::Field_<ContainerAllocator>>> & _arg)
  {
    this->fields = _arg;
    return *this;
  }
  Type & set__device_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->device_name = _arg;
    return *this;
  }
  Type & set__monitoring_cases(
    const std::vector<sick_safetyscanners2_interfaces::msg::MonitoringCase_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sick_safetyscanners2_interfaces::msg::MonitoringCase_<ContainerAllocator>>> & _arg)
  {
    this->monitoring_cases = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sick_safetyscanners2_interfaces::srv::FieldData_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const sick_safetyscanners2_interfaces::srv::FieldData_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::srv::FieldData_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::srv::FieldData_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::srv::FieldData_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::srv::FieldData_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::srv::FieldData_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::srv::FieldData_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::srv::FieldData_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::srv::FieldData_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sick_safetyscanners2_interfaces__srv__FieldData_Response
    std::shared_ptr<sick_safetyscanners2_interfaces::srv::FieldData_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sick_safetyscanners2_interfaces__srv__FieldData_Response
    std::shared_ptr<sick_safetyscanners2_interfaces::srv::FieldData_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FieldData_Response_ & other) const
  {
    if (this->fields != other.fields) {
      return false;
    }
    if (this->device_name != other.device_name) {
      return false;
    }
    if (this->monitoring_cases != other.monitoring_cases) {
      return false;
    }
    return true;
  }
  bool operator!=(const FieldData_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FieldData_Response_

// alias to use template instance with default allocator
using FieldData_Response =
  sick_safetyscanners2_interfaces::srv::FieldData_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace sick_safetyscanners2_interfaces

namespace sick_safetyscanners2_interfaces
{

namespace srv
{

struct FieldData
{
  using Request = sick_safetyscanners2_interfaces::srv::FieldData_Request;
  using Response = sick_safetyscanners2_interfaces::srv::FieldData_Response;
};

}  // namespace srv

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__SRV__DETAIL__FIELD_DATA__STRUCT_HPP_
