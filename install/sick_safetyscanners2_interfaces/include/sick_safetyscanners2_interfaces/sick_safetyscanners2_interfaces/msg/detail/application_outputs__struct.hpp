// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/ApplicationOutputs.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_OUTPUTS__STRUCT_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_OUTPUTS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__sick_safetyscanners2_interfaces__msg__ApplicationOutputs __attribute__((deprecated))
#else
# define DEPRECATED__sick_safetyscanners2_interfaces__msg__ApplicationOutputs __declspec(deprecated)
#endif

namespace sick_safetyscanners2_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ApplicationOutputs_
{
  using Type = ApplicationOutputs_<ContainerAllocator>;

  explicit ApplicationOutputs_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sleep_mode_output = 0;
      this->sleep_mode_output_valid = false;
      this->error_flag_contamination_warning = false;
      this->error_flag_contamination_error = false;
      this->error_flag_manipulation_error = false;
      this->error_flag_glare = false;
      this->error_flag_reference_contour_intruded = false;
      this->error_flag_critical_error = false;
      this->error_flags_are_valid = false;
      this->linear_velocity_outputs_velocity_0 = 0;
      this->linear_velocity_outputs_velocity_0_valid = false;
      this->linear_velocity_outputs_velocity_0_transmitted_safely = false;
      this->linear_velocity_outputs_velocity_1 = 0;
      this->linear_velocity_outputs_velocity_1_valid = false;
      this->linear_velocity_outputs_velocity_1_transmitted_safely = false;
    }
  }

  explicit ApplicationOutputs_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sleep_mode_output = 0;
      this->sleep_mode_output_valid = false;
      this->error_flag_contamination_warning = false;
      this->error_flag_contamination_error = false;
      this->error_flag_manipulation_error = false;
      this->error_flag_glare = false;
      this->error_flag_reference_contour_intruded = false;
      this->error_flag_critical_error = false;
      this->error_flags_are_valid = false;
      this->linear_velocity_outputs_velocity_0 = 0;
      this->linear_velocity_outputs_velocity_0_valid = false;
      this->linear_velocity_outputs_velocity_0_transmitted_safely = false;
      this->linear_velocity_outputs_velocity_1 = 0;
      this->linear_velocity_outputs_velocity_1_valid = false;
      this->linear_velocity_outputs_velocity_1_transmitted_safely = false;
    }
  }

  // field types and members
  using _evaluation_path_outputs_eval_out_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _evaluation_path_outputs_eval_out_type evaluation_path_outputs_eval_out;
  using _evaluation_path_outputs_is_safe_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _evaluation_path_outputs_is_safe_type evaluation_path_outputs_is_safe;
  using _evaluation_path_outputs_is_valid_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _evaluation_path_outputs_is_valid_type evaluation_path_outputs_is_valid;
  using _monitoring_case_number_outputs_type =
    std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>>;
  _monitoring_case_number_outputs_type monitoring_case_number_outputs;
  using _monitoring_case_number_outputs_flags_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _monitoring_case_number_outputs_flags_type monitoring_case_number_outputs_flags;
  using _sleep_mode_output_type =
    uint8_t;
  _sleep_mode_output_type sleep_mode_output;
  using _sleep_mode_output_valid_type =
    bool;
  _sleep_mode_output_valid_type sleep_mode_output_valid;
  using _error_flag_contamination_warning_type =
    bool;
  _error_flag_contamination_warning_type error_flag_contamination_warning;
  using _error_flag_contamination_error_type =
    bool;
  _error_flag_contamination_error_type error_flag_contamination_error;
  using _error_flag_manipulation_error_type =
    bool;
  _error_flag_manipulation_error_type error_flag_manipulation_error;
  using _error_flag_glare_type =
    bool;
  _error_flag_glare_type error_flag_glare;
  using _error_flag_reference_contour_intruded_type =
    bool;
  _error_flag_reference_contour_intruded_type error_flag_reference_contour_intruded;
  using _error_flag_critical_error_type =
    bool;
  _error_flag_critical_error_type error_flag_critical_error;
  using _error_flags_are_valid_type =
    bool;
  _error_flags_are_valid_type error_flags_are_valid;
  using _linear_velocity_outputs_velocity_0_type =
    int16_t;
  _linear_velocity_outputs_velocity_0_type linear_velocity_outputs_velocity_0;
  using _linear_velocity_outputs_velocity_0_valid_type =
    bool;
  _linear_velocity_outputs_velocity_0_valid_type linear_velocity_outputs_velocity_0_valid;
  using _linear_velocity_outputs_velocity_0_transmitted_safely_type =
    bool;
  _linear_velocity_outputs_velocity_0_transmitted_safely_type linear_velocity_outputs_velocity_0_transmitted_safely;
  using _linear_velocity_outputs_velocity_1_type =
    int16_t;
  _linear_velocity_outputs_velocity_1_type linear_velocity_outputs_velocity_1;
  using _linear_velocity_outputs_velocity_1_valid_type =
    bool;
  _linear_velocity_outputs_velocity_1_valid_type linear_velocity_outputs_velocity_1_valid;
  using _linear_velocity_outputs_velocity_1_transmitted_safely_type =
    bool;
  _linear_velocity_outputs_velocity_1_transmitted_safely_type linear_velocity_outputs_velocity_1_transmitted_safely;
  using _resulting_velocity_type =
    std::vector<int16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int16_t>>;
  _resulting_velocity_type resulting_velocity;
  using _resulting_velocity_flags_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _resulting_velocity_flags_type resulting_velocity_flags;

  // setters for named parameter idiom
  Type & set__evaluation_path_outputs_eval_out(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->evaluation_path_outputs_eval_out = _arg;
    return *this;
  }
  Type & set__evaluation_path_outputs_is_safe(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->evaluation_path_outputs_is_safe = _arg;
    return *this;
  }
  Type & set__evaluation_path_outputs_is_valid(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->evaluation_path_outputs_is_valid = _arg;
    return *this;
  }
  Type & set__monitoring_case_number_outputs(
    const std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>> & _arg)
  {
    this->monitoring_case_number_outputs = _arg;
    return *this;
  }
  Type & set__monitoring_case_number_outputs_flags(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->monitoring_case_number_outputs_flags = _arg;
    return *this;
  }
  Type & set__sleep_mode_output(
    const uint8_t & _arg)
  {
    this->sleep_mode_output = _arg;
    return *this;
  }
  Type & set__sleep_mode_output_valid(
    const bool & _arg)
  {
    this->sleep_mode_output_valid = _arg;
    return *this;
  }
  Type & set__error_flag_contamination_warning(
    const bool & _arg)
  {
    this->error_flag_contamination_warning = _arg;
    return *this;
  }
  Type & set__error_flag_contamination_error(
    const bool & _arg)
  {
    this->error_flag_contamination_error = _arg;
    return *this;
  }
  Type & set__error_flag_manipulation_error(
    const bool & _arg)
  {
    this->error_flag_manipulation_error = _arg;
    return *this;
  }
  Type & set__error_flag_glare(
    const bool & _arg)
  {
    this->error_flag_glare = _arg;
    return *this;
  }
  Type & set__error_flag_reference_contour_intruded(
    const bool & _arg)
  {
    this->error_flag_reference_contour_intruded = _arg;
    return *this;
  }
  Type & set__error_flag_critical_error(
    const bool & _arg)
  {
    this->error_flag_critical_error = _arg;
    return *this;
  }
  Type & set__error_flags_are_valid(
    const bool & _arg)
  {
    this->error_flags_are_valid = _arg;
    return *this;
  }
  Type & set__linear_velocity_outputs_velocity_0(
    const int16_t & _arg)
  {
    this->linear_velocity_outputs_velocity_0 = _arg;
    return *this;
  }
  Type & set__linear_velocity_outputs_velocity_0_valid(
    const bool & _arg)
  {
    this->linear_velocity_outputs_velocity_0_valid = _arg;
    return *this;
  }
  Type & set__linear_velocity_outputs_velocity_0_transmitted_safely(
    const bool & _arg)
  {
    this->linear_velocity_outputs_velocity_0_transmitted_safely = _arg;
    return *this;
  }
  Type & set__linear_velocity_outputs_velocity_1(
    const int16_t & _arg)
  {
    this->linear_velocity_outputs_velocity_1 = _arg;
    return *this;
  }
  Type & set__linear_velocity_outputs_velocity_1_valid(
    const bool & _arg)
  {
    this->linear_velocity_outputs_velocity_1_valid = _arg;
    return *this;
  }
  Type & set__linear_velocity_outputs_velocity_1_transmitted_safely(
    const bool & _arg)
  {
    this->linear_velocity_outputs_velocity_1_transmitted_safely = _arg;
    return *this;
  }
  Type & set__resulting_velocity(
    const std::vector<int16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int16_t>> & _arg)
  {
    this->resulting_velocity = _arg;
    return *this;
  }
  Type & set__resulting_velocity_flags(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->resulting_velocity_flags = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sick_safetyscanners2_interfaces::msg::ApplicationOutputs_<ContainerAllocator> *;
  using ConstRawPtr =
    const sick_safetyscanners2_interfaces::msg::ApplicationOutputs_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::ApplicationOutputs_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::ApplicationOutputs_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::msg::ApplicationOutputs_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::msg::ApplicationOutputs_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::msg::ApplicationOutputs_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::msg::ApplicationOutputs_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::msg::ApplicationOutputs_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::msg::ApplicationOutputs_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sick_safetyscanners2_interfaces__msg__ApplicationOutputs
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::ApplicationOutputs_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sick_safetyscanners2_interfaces__msg__ApplicationOutputs
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::ApplicationOutputs_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ApplicationOutputs_ & other) const
  {
    if (this->evaluation_path_outputs_eval_out != other.evaluation_path_outputs_eval_out) {
      return false;
    }
    if (this->evaluation_path_outputs_is_safe != other.evaluation_path_outputs_is_safe) {
      return false;
    }
    if (this->evaluation_path_outputs_is_valid != other.evaluation_path_outputs_is_valid) {
      return false;
    }
    if (this->monitoring_case_number_outputs != other.monitoring_case_number_outputs) {
      return false;
    }
    if (this->monitoring_case_number_outputs_flags != other.monitoring_case_number_outputs_flags) {
      return false;
    }
    if (this->sleep_mode_output != other.sleep_mode_output) {
      return false;
    }
    if (this->sleep_mode_output_valid != other.sleep_mode_output_valid) {
      return false;
    }
    if (this->error_flag_contamination_warning != other.error_flag_contamination_warning) {
      return false;
    }
    if (this->error_flag_contamination_error != other.error_flag_contamination_error) {
      return false;
    }
    if (this->error_flag_manipulation_error != other.error_flag_manipulation_error) {
      return false;
    }
    if (this->error_flag_glare != other.error_flag_glare) {
      return false;
    }
    if (this->error_flag_reference_contour_intruded != other.error_flag_reference_contour_intruded) {
      return false;
    }
    if (this->error_flag_critical_error != other.error_flag_critical_error) {
      return false;
    }
    if (this->error_flags_are_valid != other.error_flags_are_valid) {
      return false;
    }
    if (this->linear_velocity_outputs_velocity_0 != other.linear_velocity_outputs_velocity_0) {
      return false;
    }
    if (this->linear_velocity_outputs_velocity_0_valid != other.linear_velocity_outputs_velocity_0_valid) {
      return false;
    }
    if (this->linear_velocity_outputs_velocity_0_transmitted_safely != other.linear_velocity_outputs_velocity_0_transmitted_safely) {
      return false;
    }
    if (this->linear_velocity_outputs_velocity_1 != other.linear_velocity_outputs_velocity_1) {
      return false;
    }
    if (this->linear_velocity_outputs_velocity_1_valid != other.linear_velocity_outputs_velocity_1_valid) {
      return false;
    }
    if (this->linear_velocity_outputs_velocity_1_transmitted_safely != other.linear_velocity_outputs_velocity_1_transmitted_safely) {
      return false;
    }
    if (this->resulting_velocity != other.resulting_velocity) {
      return false;
    }
    if (this->resulting_velocity_flags != other.resulting_velocity_flags) {
      return false;
    }
    return true;
  }
  bool operator!=(const ApplicationOutputs_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ApplicationOutputs_

// alias to use template instance with default allocator
using ApplicationOutputs =
  sick_safetyscanners2_interfaces::msg::ApplicationOutputs_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_OUTPUTS__STRUCT_HPP_
