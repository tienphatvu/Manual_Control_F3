// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/ScanPoint.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__SCAN_POINT__STRUCT_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__SCAN_POINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__sick_safetyscanners2_interfaces__msg__ScanPoint __attribute__((deprecated))
#else
# define DEPRECATED__sick_safetyscanners2_interfaces__msg__ScanPoint __declspec(deprecated)
#endif

namespace sick_safetyscanners2_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ScanPoint_
{
  using Type = ScanPoint_<ContainerAllocator>;

  explicit ScanPoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angle = 0.0f;
      this->distance = 0;
      this->reflectivity = 0;
      this->valid = false;
      this->infinite = false;
      this->glare = false;
      this->reflector = false;
      this->contamination = false;
      this->contamination_warning = false;
    }
  }

  explicit ScanPoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angle = 0.0f;
      this->distance = 0;
      this->reflectivity = 0;
      this->valid = false;
      this->infinite = false;
      this->glare = false;
      this->reflector = false;
      this->contamination = false;
      this->contamination_warning = false;
    }
  }

  // field types and members
  using _angle_type =
    float;
  _angle_type angle;
  using _distance_type =
    uint16_t;
  _distance_type distance;
  using _reflectivity_type =
    uint8_t;
  _reflectivity_type reflectivity;
  using _valid_type =
    bool;
  _valid_type valid;
  using _infinite_type =
    bool;
  _infinite_type infinite;
  using _glare_type =
    bool;
  _glare_type glare;
  using _reflector_type =
    bool;
  _reflector_type reflector;
  using _contamination_type =
    bool;
  _contamination_type contamination;
  using _contamination_warning_type =
    bool;
  _contamination_warning_type contamination_warning;

  // setters for named parameter idiom
  Type & set__angle(
    const float & _arg)
  {
    this->angle = _arg;
    return *this;
  }
  Type & set__distance(
    const uint16_t & _arg)
  {
    this->distance = _arg;
    return *this;
  }
  Type & set__reflectivity(
    const uint8_t & _arg)
  {
    this->reflectivity = _arg;
    return *this;
  }
  Type & set__valid(
    const bool & _arg)
  {
    this->valid = _arg;
    return *this;
  }
  Type & set__infinite(
    const bool & _arg)
  {
    this->infinite = _arg;
    return *this;
  }
  Type & set__glare(
    const bool & _arg)
  {
    this->glare = _arg;
    return *this;
  }
  Type & set__reflector(
    const bool & _arg)
  {
    this->reflector = _arg;
    return *this;
  }
  Type & set__contamination(
    const bool & _arg)
  {
    this->contamination = _arg;
    return *this;
  }
  Type & set__contamination_warning(
    const bool & _arg)
  {
    this->contamination_warning = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sick_safetyscanners2_interfaces::msg::ScanPoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const sick_safetyscanners2_interfaces::msg::ScanPoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::ScanPoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::ScanPoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::msg::ScanPoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::msg::ScanPoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::msg::ScanPoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::msg::ScanPoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::msg::ScanPoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::msg::ScanPoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sick_safetyscanners2_interfaces__msg__ScanPoint
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::ScanPoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sick_safetyscanners2_interfaces__msg__ScanPoint
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::ScanPoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ScanPoint_ & other) const
  {
    if (this->angle != other.angle) {
      return false;
    }
    if (this->distance != other.distance) {
      return false;
    }
    if (this->reflectivity != other.reflectivity) {
      return false;
    }
    if (this->valid != other.valid) {
      return false;
    }
    if (this->infinite != other.infinite) {
      return false;
    }
    if (this->glare != other.glare) {
      return false;
    }
    if (this->reflector != other.reflector) {
      return false;
    }
    if (this->contamination != other.contamination) {
      return false;
    }
    if (this->contamination_warning != other.contamination_warning) {
      return false;
    }
    return true;
  }
  bool operator!=(const ScanPoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ScanPoint_

// alias to use template instance with default allocator
using ScanPoint =
  sick_safetyscanners2_interfaces::msg::ScanPoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__SCAN_POINT__STRUCT_HPP_
