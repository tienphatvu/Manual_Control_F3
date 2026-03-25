// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/MonitoringCase.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__MONITORING_CASE__STRUCT_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__MONITORING_CASE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__sick_safetyscanners2_interfaces__msg__MonitoringCase __attribute__((deprecated))
#else
# define DEPRECATED__sick_safetyscanners2_interfaces__msg__MonitoringCase __declspec(deprecated)
#endif

namespace sick_safetyscanners2_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MonitoringCase_
{
  using Type = MonitoringCase_<ContainerAllocator>;

  explicit MonitoringCase_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->monitoring_case_number = 0l;
    }
  }

  explicit MonitoringCase_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->monitoring_case_number = 0l;
    }
  }

  // field types and members
  using _monitoring_case_number_type =
    int32_t;
  _monitoring_case_number_type monitoring_case_number;
  using _fields_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _fields_type fields;
  using _fields_valid_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _fields_valid_type fields_valid;

  // setters for named parameter idiom
  Type & set__monitoring_case_number(
    const int32_t & _arg)
  {
    this->monitoring_case_number = _arg;
    return *this;
  }
  Type & set__fields(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->fields = _arg;
    return *this;
  }
  Type & set__fields_valid(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->fields_valid = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sick_safetyscanners2_interfaces::msg::MonitoringCase_<ContainerAllocator> *;
  using ConstRawPtr =
    const sick_safetyscanners2_interfaces::msg::MonitoringCase_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::MonitoringCase_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::MonitoringCase_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::msg::MonitoringCase_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::msg::MonitoringCase_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::msg::MonitoringCase_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::msg::MonitoringCase_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::msg::MonitoringCase_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::msg::MonitoringCase_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sick_safetyscanners2_interfaces__msg__MonitoringCase
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::MonitoringCase_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sick_safetyscanners2_interfaces__msg__MonitoringCase
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::MonitoringCase_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MonitoringCase_ & other) const
  {
    if (this->monitoring_case_number != other.monitoring_case_number) {
      return false;
    }
    if (this->fields != other.fields) {
      return false;
    }
    if (this->fields_valid != other.fields_valid) {
      return false;
    }
    return true;
  }
  bool operator!=(const MonitoringCase_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MonitoringCase_

// alias to use template instance with default allocator
using MonitoringCase =
  sick_safetyscanners2_interfaces::msg::MonitoringCase_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__MONITORING_CASE__STRUCT_HPP_
