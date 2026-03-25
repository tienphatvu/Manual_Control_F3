// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/GeneralSystemState.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__GENERAL_SYSTEM_STATE__STRUCT_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__GENERAL_SYSTEM_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__sick_safetyscanners2_interfaces__msg__GeneralSystemState __attribute__((deprecated))
#else
# define DEPRECATED__sick_safetyscanners2_interfaces__msg__GeneralSystemState __declspec(deprecated)
#endif

namespace sick_safetyscanners2_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GeneralSystemState_
{
  using Type = GeneralSystemState_<ContainerAllocator>;

  explicit GeneralSystemState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->run_mode_active = false;
      this->standby_mode_active = false;
      this->contamination_warning = false;
      this->contamination_error = false;
      this->reference_contour_status = false;
      this->manipulation_status = false;
      this->current_monitoring_case_no_table_1 = 0;
      this->current_monitoring_case_no_table_2 = 0;
      this->current_monitoring_case_no_table_3 = 0;
      this->current_monitoring_case_no_table_4 = 0;
      this->application_error = false;
      this->device_error = false;
    }
  }

  explicit GeneralSystemState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->run_mode_active = false;
      this->standby_mode_active = false;
      this->contamination_warning = false;
      this->contamination_error = false;
      this->reference_contour_status = false;
      this->manipulation_status = false;
      this->current_monitoring_case_no_table_1 = 0;
      this->current_monitoring_case_no_table_2 = 0;
      this->current_monitoring_case_no_table_3 = 0;
      this->current_monitoring_case_no_table_4 = 0;
      this->application_error = false;
      this->device_error = false;
    }
  }

  // field types and members
  using _run_mode_active_type =
    bool;
  _run_mode_active_type run_mode_active;
  using _standby_mode_active_type =
    bool;
  _standby_mode_active_type standby_mode_active;
  using _contamination_warning_type =
    bool;
  _contamination_warning_type contamination_warning;
  using _contamination_error_type =
    bool;
  _contamination_error_type contamination_error;
  using _reference_contour_status_type =
    bool;
  _reference_contour_status_type reference_contour_status;
  using _manipulation_status_type =
    bool;
  _manipulation_status_type manipulation_status;
  using _safe_cut_off_path_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _safe_cut_off_path_type safe_cut_off_path;
  using _non_safe_cut_off_path_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _non_safe_cut_off_path_type non_safe_cut_off_path;
  using _reset_required_cut_off_path_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _reset_required_cut_off_path_type reset_required_cut_off_path;
  using _current_monitoring_case_no_table_1_type =
    uint8_t;
  _current_monitoring_case_no_table_1_type current_monitoring_case_no_table_1;
  using _current_monitoring_case_no_table_2_type =
    uint8_t;
  _current_monitoring_case_no_table_2_type current_monitoring_case_no_table_2;
  using _current_monitoring_case_no_table_3_type =
    uint8_t;
  _current_monitoring_case_no_table_3_type current_monitoring_case_no_table_3;
  using _current_monitoring_case_no_table_4_type =
    uint8_t;
  _current_monitoring_case_no_table_4_type current_monitoring_case_no_table_4;
  using _application_error_type =
    bool;
  _application_error_type application_error;
  using _device_error_type =
    bool;
  _device_error_type device_error;

  // setters for named parameter idiom
  Type & set__run_mode_active(
    const bool & _arg)
  {
    this->run_mode_active = _arg;
    return *this;
  }
  Type & set__standby_mode_active(
    const bool & _arg)
  {
    this->standby_mode_active = _arg;
    return *this;
  }
  Type & set__contamination_warning(
    const bool & _arg)
  {
    this->contamination_warning = _arg;
    return *this;
  }
  Type & set__contamination_error(
    const bool & _arg)
  {
    this->contamination_error = _arg;
    return *this;
  }
  Type & set__reference_contour_status(
    const bool & _arg)
  {
    this->reference_contour_status = _arg;
    return *this;
  }
  Type & set__manipulation_status(
    const bool & _arg)
  {
    this->manipulation_status = _arg;
    return *this;
  }
  Type & set__safe_cut_off_path(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->safe_cut_off_path = _arg;
    return *this;
  }
  Type & set__non_safe_cut_off_path(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->non_safe_cut_off_path = _arg;
    return *this;
  }
  Type & set__reset_required_cut_off_path(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->reset_required_cut_off_path = _arg;
    return *this;
  }
  Type & set__current_monitoring_case_no_table_1(
    const uint8_t & _arg)
  {
    this->current_monitoring_case_no_table_1 = _arg;
    return *this;
  }
  Type & set__current_monitoring_case_no_table_2(
    const uint8_t & _arg)
  {
    this->current_monitoring_case_no_table_2 = _arg;
    return *this;
  }
  Type & set__current_monitoring_case_no_table_3(
    const uint8_t & _arg)
  {
    this->current_monitoring_case_no_table_3 = _arg;
    return *this;
  }
  Type & set__current_monitoring_case_no_table_4(
    const uint8_t & _arg)
  {
    this->current_monitoring_case_no_table_4 = _arg;
    return *this;
  }
  Type & set__application_error(
    const bool & _arg)
  {
    this->application_error = _arg;
    return *this;
  }
  Type & set__device_error(
    const bool & _arg)
  {
    this->device_error = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sick_safetyscanners2_interfaces::msg::GeneralSystemState_<ContainerAllocator> *;
  using ConstRawPtr =
    const sick_safetyscanners2_interfaces::msg::GeneralSystemState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::GeneralSystemState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::GeneralSystemState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::msg::GeneralSystemState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::msg::GeneralSystemState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::msg::GeneralSystemState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::msg::GeneralSystemState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::msg::GeneralSystemState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::msg::GeneralSystemState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sick_safetyscanners2_interfaces__msg__GeneralSystemState
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::GeneralSystemState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sick_safetyscanners2_interfaces__msg__GeneralSystemState
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::GeneralSystemState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GeneralSystemState_ & other) const
  {
    if (this->run_mode_active != other.run_mode_active) {
      return false;
    }
    if (this->standby_mode_active != other.standby_mode_active) {
      return false;
    }
    if (this->contamination_warning != other.contamination_warning) {
      return false;
    }
    if (this->contamination_error != other.contamination_error) {
      return false;
    }
    if (this->reference_contour_status != other.reference_contour_status) {
      return false;
    }
    if (this->manipulation_status != other.manipulation_status) {
      return false;
    }
    if (this->safe_cut_off_path != other.safe_cut_off_path) {
      return false;
    }
    if (this->non_safe_cut_off_path != other.non_safe_cut_off_path) {
      return false;
    }
    if (this->reset_required_cut_off_path != other.reset_required_cut_off_path) {
      return false;
    }
    if (this->current_monitoring_case_no_table_1 != other.current_monitoring_case_no_table_1) {
      return false;
    }
    if (this->current_monitoring_case_no_table_2 != other.current_monitoring_case_no_table_2) {
      return false;
    }
    if (this->current_monitoring_case_no_table_3 != other.current_monitoring_case_no_table_3) {
      return false;
    }
    if (this->current_monitoring_case_no_table_4 != other.current_monitoring_case_no_table_4) {
      return false;
    }
    if (this->application_error != other.application_error) {
      return false;
    }
    if (this->device_error != other.device_error) {
      return false;
    }
    return true;
  }
  bool operator!=(const GeneralSystemState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GeneralSystemState_

// alias to use template instance with default allocator
using GeneralSystemState =
  sick_safetyscanners2_interfaces::msg::GeneralSystemState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__GENERAL_SYSTEM_STATE__STRUCT_HPP_
