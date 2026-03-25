// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/DerivedValues.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__DERIVED_VALUES__STRUCT_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__DERIVED_VALUES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__sick_safetyscanners2_interfaces__msg__DerivedValues __attribute__((deprecated))
#else
# define DEPRECATED__sick_safetyscanners2_interfaces__msg__DerivedValues __declspec(deprecated)
#endif

namespace sick_safetyscanners2_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DerivedValues_
{
  using Type = DerivedValues_<ContainerAllocator>;

  explicit DerivedValues_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->multiplication_factor = 0;
      this->number_of_beams = 0;
      this->scan_time = 0;
      this->start_angle = 0.0f;
      this->angular_beam_resolution = 0.0f;
      this->interbeam_period = 0ul;
    }
  }

  explicit DerivedValues_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->multiplication_factor = 0;
      this->number_of_beams = 0;
      this->scan_time = 0;
      this->start_angle = 0.0f;
      this->angular_beam_resolution = 0.0f;
      this->interbeam_period = 0ul;
    }
  }

  // field types and members
  using _multiplication_factor_type =
    uint16_t;
  _multiplication_factor_type multiplication_factor;
  using _number_of_beams_type =
    uint16_t;
  _number_of_beams_type number_of_beams;
  using _scan_time_type =
    uint16_t;
  _scan_time_type scan_time;
  using _start_angle_type =
    float;
  _start_angle_type start_angle;
  using _angular_beam_resolution_type =
    float;
  _angular_beam_resolution_type angular_beam_resolution;
  using _interbeam_period_type =
    uint32_t;
  _interbeam_period_type interbeam_period;

  // setters for named parameter idiom
  Type & set__multiplication_factor(
    const uint16_t & _arg)
  {
    this->multiplication_factor = _arg;
    return *this;
  }
  Type & set__number_of_beams(
    const uint16_t & _arg)
  {
    this->number_of_beams = _arg;
    return *this;
  }
  Type & set__scan_time(
    const uint16_t & _arg)
  {
    this->scan_time = _arg;
    return *this;
  }
  Type & set__start_angle(
    const float & _arg)
  {
    this->start_angle = _arg;
    return *this;
  }
  Type & set__angular_beam_resolution(
    const float & _arg)
  {
    this->angular_beam_resolution = _arg;
    return *this;
  }
  Type & set__interbeam_period(
    const uint32_t & _arg)
  {
    this->interbeam_period = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sick_safetyscanners2_interfaces::msg::DerivedValues_<ContainerAllocator> *;
  using ConstRawPtr =
    const sick_safetyscanners2_interfaces::msg::DerivedValues_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::DerivedValues_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::DerivedValues_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::msg::DerivedValues_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::msg::DerivedValues_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::msg::DerivedValues_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::msg::DerivedValues_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::msg::DerivedValues_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::msg::DerivedValues_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sick_safetyscanners2_interfaces__msg__DerivedValues
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::DerivedValues_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sick_safetyscanners2_interfaces__msg__DerivedValues
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::DerivedValues_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DerivedValues_ & other) const
  {
    if (this->multiplication_factor != other.multiplication_factor) {
      return false;
    }
    if (this->number_of_beams != other.number_of_beams) {
      return false;
    }
    if (this->scan_time != other.scan_time) {
      return false;
    }
    if (this->start_angle != other.start_angle) {
      return false;
    }
    if (this->angular_beam_resolution != other.angular_beam_resolution) {
      return false;
    }
    if (this->interbeam_period != other.interbeam_period) {
      return false;
    }
    return true;
  }
  bool operator!=(const DerivedValues_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DerivedValues_

// alias to use template instance with default allocator
using DerivedValues =
  sick_safetyscanners2_interfaces::msg::DerivedValues_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__DERIVED_VALUES__STRUCT_HPP_
