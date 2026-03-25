// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/ApplicationInputs.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_INPUTS__STRUCT_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_INPUTS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__sick_safetyscanners2_interfaces__msg__ApplicationInputs __attribute__((deprecated))
#else
# define DEPRECATED__sick_safetyscanners2_interfaces__msg__ApplicationInputs __declspec(deprecated)
#endif

namespace sick_safetyscanners2_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ApplicationInputs_
{
  using Type = ApplicationInputs_<ContainerAllocator>;

  explicit ApplicationInputs_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->linear_velocity_inputs_velocity_0 = 0;
      this->linear_velocity_inputs_velocity_0_valid = false;
      this->linear_velocity_inputs_velocity_0_transmitted_safely = false;
      this->linear_velocity_inputs_velocity_1 = 0;
      this->linear_velocity_inputs_velocity_1_valid = false;
      this->linear_velocity_inputs_velocity_1_transmitted_safely = false;
      this->sleep_mode_input = 0;
    }
  }

  explicit ApplicationInputs_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->linear_velocity_inputs_velocity_0 = 0;
      this->linear_velocity_inputs_velocity_0_valid = false;
      this->linear_velocity_inputs_velocity_0_transmitted_safely = false;
      this->linear_velocity_inputs_velocity_1 = 0;
      this->linear_velocity_inputs_velocity_1_valid = false;
      this->linear_velocity_inputs_velocity_1_transmitted_safely = false;
      this->sleep_mode_input = 0;
    }
  }

  // field types and members
  using _unsafe_inputs_input_sources_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _unsafe_inputs_input_sources_type unsafe_inputs_input_sources;
  using _unsafe_inputs_flags_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _unsafe_inputs_flags_type unsafe_inputs_flags;
  using _monitoring_case_number_inputs_type =
    std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>>;
  _monitoring_case_number_inputs_type monitoring_case_number_inputs;
  using _monitoring_case_number_inputs_flags_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _monitoring_case_number_inputs_flags_type monitoring_case_number_inputs_flags;
  using _linear_velocity_inputs_velocity_0_type =
    int16_t;
  _linear_velocity_inputs_velocity_0_type linear_velocity_inputs_velocity_0;
  using _linear_velocity_inputs_velocity_0_valid_type =
    bool;
  _linear_velocity_inputs_velocity_0_valid_type linear_velocity_inputs_velocity_0_valid;
  using _linear_velocity_inputs_velocity_0_transmitted_safely_type =
    bool;
  _linear_velocity_inputs_velocity_0_transmitted_safely_type linear_velocity_inputs_velocity_0_transmitted_safely;
  using _linear_velocity_inputs_velocity_1_type =
    int16_t;
  _linear_velocity_inputs_velocity_1_type linear_velocity_inputs_velocity_1;
  using _linear_velocity_inputs_velocity_1_valid_type =
    bool;
  _linear_velocity_inputs_velocity_1_valid_type linear_velocity_inputs_velocity_1_valid;
  using _linear_velocity_inputs_velocity_1_transmitted_safely_type =
    bool;
  _linear_velocity_inputs_velocity_1_transmitted_safely_type linear_velocity_inputs_velocity_1_transmitted_safely;
  using _sleep_mode_input_type =
    uint8_t;
  _sleep_mode_input_type sleep_mode_input;

  // setters for named parameter idiom
  Type & set__unsafe_inputs_input_sources(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->unsafe_inputs_input_sources = _arg;
    return *this;
  }
  Type & set__unsafe_inputs_flags(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->unsafe_inputs_flags = _arg;
    return *this;
  }
  Type & set__monitoring_case_number_inputs(
    const std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>> & _arg)
  {
    this->monitoring_case_number_inputs = _arg;
    return *this;
  }
  Type & set__monitoring_case_number_inputs_flags(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->monitoring_case_number_inputs_flags = _arg;
    return *this;
  }
  Type & set__linear_velocity_inputs_velocity_0(
    const int16_t & _arg)
  {
    this->linear_velocity_inputs_velocity_0 = _arg;
    return *this;
  }
  Type & set__linear_velocity_inputs_velocity_0_valid(
    const bool & _arg)
  {
    this->linear_velocity_inputs_velocity_0_valid = _arg;
    return *this;
  }
  Type & set__linear_velocity_inputs_velocity_0_transmitted_safely(
    const bool & _arg)
  {
    this->linear_velocity_inputs_velocity_0_transmitted_safely = _arg;
    return *this;
  }
  Type & set__linear_velocity_inputs_velocity_1(
    const int16_t & _arg)
  {
    this->linear_velocity_inputs_velocity_1 = _arg;
    return *this;
  }
  Type & set__linear_velocity_inputs_velocity_1_valid(
    const bool & _arg)
  {
    this->linear_velocity_inputs_velocity_1_valid = _arg;
    return *this;
  }
  Type & set__linear_velocity_inputs_velocity_1_transmitted_safely(
    const bool & _arg)
  {
    this->linear_velocity_inputs_velocity_1_transmitted_safely = _arg;
    return *this;
  }
  Type & set__sleep_mode_input(
    const uint8_t & _arg)
  {
    this->sleep_mode_input = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sick_safetyscanners2_interfaces::msg::ApplicationInputs_<ContainerAllocator> *;
  using ConstRawPtr =
    const sick_safetyscanners2_interfaces::msg::ApplicationInputs_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::ApplicationInputs_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::ApplicationInputs_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::msg::ApplicationInputs_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::msg::ApplicationInputs_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::msg::ApplicationInputs_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::msg::ApplicationInputs_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::msg::ApplicationInputs_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::msg::ApplicationInputs_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sick_safetyscanners2_interfaces__msg__ApplicationInputs
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::ApplicationInputs_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sick_safetyscanners2_interfaces__msg__ApplicationInputs
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::ApplicationInputs_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ApplicationInputs_ & other) const
  {
    if (this->unsafe_inputs_input_sources != other.unsafe_inputs_input_sources) {
      return false;
    }
    if (this->unsafe_inputs_flags != other.unsafe_inputs_flags) {
      return false;
    }
    if (this->monitoring_case_number_inputs != other.monitoring_case_number_inputs) {
      return false;
    }
    if (this->monitoring_case_number_inputs_flags != other.monitoring_case_number_inputs_flags) {
      return false;
    }
    if (this->linear_velocity_inputs_velocity_0 != other.linear_velocity_inputs_velocity_0) {
      return false;
    }
    if (this->linear_velocity_inputs_velocity_0_valid != other.linear_velocity_inputs_velocity_0_valid) {
      return false;
    }
    if (this->linear_velocity_inputs_velocity_0_transmitted_safely != other.linear_velocity_inputs_velocity_0_transmitted_safely) {
      return false;
    }
    if (this->linear_velocity_inputs_velocity_1 != other.linear_velocity_inputs_velocity_1) {
      return false;
    }
    if (this->linear_velocity_inputs_velocity_1_valid != other.linear_velocity_inputs_velocity_1_valid) {
      return false;
    }
    if (this->linear_velocity_inputs_velocity_1_transmitted_safely != other.linear_velocity_inputs_velocity_1_transmitted_safely) {
      return false;
    }
    if (this->sleep_mode_input != other.sleep_mode_input) {
      return false;
    }
    return true;
  }
  bool operator!=(const ApplicationInputs_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ApplicationInputs_

// alias to use template instance with default allocator
using ApplicationInputs =
  sick_safetyscanners2_interfaces::msg::ApplicationInputs_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_INPUTS__STRUCT_HPP_
