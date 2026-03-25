// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/OutputPaths.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__OUTPUT_PATHS__STRUCT_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__OUTPUT_PATHS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__sick_safetyscanners2_interfaces__msg__OutputPaths __attribute__((deprecated))
#else
# define DEPRECATED__sick_safetyscanners2_interfaces__msg__OutputPaths __declspec(deprecated)
#endif

namespace sick_safetyscanners2_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OutputPaths_
{
  using Type = OutputPaths_<ContainerAllocator>;

  explicit OutputPaths_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->active_monitoring_case = 0l;
    }
  }

  explicit OutputPaths_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->active_monitoring_case = 0l;
    }
  }

  // field types and members
  using _status_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _status_type status;
  using _is_safe_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _is_safe_type is_safe;
  using _is_valid_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _is_valid_type is_valid;
  using _active_monitoring_case_type =
    int32_t;
  _active_monitoring_case_type active_monitoring_case;

  // setters for named parameter idiom
  Type & set__status(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__is_safe(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->is_safe = _arg;
    return *this;
  }
  Type & set__is_valid(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->is_valid = _arg;
    return *this;
  }
  Type & set__active_monitoring_case(
    const int32_t & _arg)
  {
    this->active_monitoring_case = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sick_safetyscanners2_interfaces::msg::OutputPaths_<ContainerAllocator> *;
  using ConstRawPtr =
    const sick_safetyscanners2_interfaces::msg::OutputPaths_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::OutputPaths_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::OutputPaths_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::msg::OutputPaths_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::msg::OutputPaths_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::msg::OutputPaths_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::msg::OutputPaths_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::msg::OutputPaths_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::msg::OutputPaths_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sick_safetyscanners2_interfaces__msg__OutputPaths
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::OutputPaths_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sick_safetyscanners2_interfaces__msg__OutputPaths
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::OutputPaths_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OutputPaths_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->is_safe != other.is_safe) {
      return false;
    }
    if (this->is_valid != other.is_valid) {
      return false;
    }
    if (this->active_monitoring_case != other.active_monitoring_case) {
      return false;
    }
    return true;
  }
  bool operator!=(const OutputPaths_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OutputPaths_

// alias to use template instance with default allocator
using OutputPaths =
  sick_safetyscanners2_interfaces::msg::OutputPaths_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__OUTPUT_PATHS__STRUCT_HPP_
