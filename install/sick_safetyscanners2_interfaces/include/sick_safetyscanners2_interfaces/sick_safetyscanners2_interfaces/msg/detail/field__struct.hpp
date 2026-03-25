// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/Field.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__FIELD__STRUCT_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__FIELD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__sick_safetyscanners2_interfaces__msg__Field __attribute__((deprecated))
#else
# define DEPRECATED__sick_safetyscanners2_interfaces__msg__Field __declspec(deprecated)
#endif

namespace sick_safetyscanners2_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Field_
{
  using Type = Field_<ContainerAllocator>;

  explicit Field_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->start_angle = 0.0f;
      this->angular_resolution = 0.0f;
      this->protective_field = false;
    }
  }

  explicit Field_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->start_angle = 0.0f;
      this->angular_resolution = 0.0f;
      this->protective_field = false;
    }
  }

  // field types and members
  using _ranges_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _ranges_type ranges;
  using _start_angle_type =
    float;
  _start_angle_type start_angle;
  using _angular_resolution_type =
    float;
  _angular_resolution_type angular_resolution;
  using _protective_field_type =
    bool;
  _protective_field_type protective_field;

  // setters for named parameter idiom
  Type & set__ranges(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->ranges = _arg;
    return *this;
  }
  Type & set__start_angle(
    const float & _arg)
  {
    this->start_angle = _arg;
    return *this;
  }
  Type & set__angular_resolution(
    const float & _arg)
  {
    this->angular_resolution = _arg;
    return *this;
  }
  Type & set__protective_field(
    const bool & _arg)
  {
    this->protective_field = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sick_safetyscanners2_interfaces::msg::Field_<ContainerAllocator> *;
  using ConstRawPtr =
    const sick_safetyscanners2_interfaces::msg::Field_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::Field_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::Field_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::msg::Field_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::msg::Field_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::msg::Field_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::msg::Field_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::msg::Field_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::msg::Field_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sick_safetyscanners2_interfaces__msg__Field
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::Field_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sick_safetyscanners2_interfaces__msg__Field
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::Field_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Field_ & other) const
  {
    if (this->ranges != other.ranges) {
      return false;
    }
    if (this->start_angle != other.start_angle) {
      return false;
    }
    if (this->angular_resolution != other.angular_resolution) {
      return false;
    }
    if (this->protective_field != other.protective_field) {
      return false;
    }
    return true;
  }
  bool operator!=(const Field_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Field_

// alias to use template instance with default allocator
using Field =
  sick_safetyscanners2_interfaces::msg::Field_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__FIELD__STRUCT_HPP_
