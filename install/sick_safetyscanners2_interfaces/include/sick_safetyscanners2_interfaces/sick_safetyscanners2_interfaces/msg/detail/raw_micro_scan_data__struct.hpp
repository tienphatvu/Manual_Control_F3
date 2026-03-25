// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/RawMicroScanData.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__RAW_MICRO_SCAN_DATA__STRUCT_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__RAW_MICRO_SCAN_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "sick_safetyscanners2_interfaces/msg/detail/data_header__struct.hpp"
// Member 'derived_values'
#include "sick_safetyscanners2_interfaces/msg/detail/derived_values__struct.hpp"
// Member 'general_system_state'
#include "sick_safetyscanners2_interfaces/msg/detail/general_system_state__struct.hpp"
// Member 'measurement_data'
#include "sick_safetyscanners2_interfaces/msg/detail/measurement_data__struct.hpp"
// Member 'intrusion_data'
#include "sick_safetyscanners2_interfaces/msg/detail/intrusion_data__struct.hpp"
// Member 'application_data'
#include "sick_safetyscanners2_interfaces/msg/detail/application_data__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__sick_safetyscanners2_interfaces__msg__RawMicroScanData __attribute__((deprecated))
#else
# define DEPRECATED__sick_safetyscanners2_interfaces__msg__RawMicroScanData __declspec(deprecated)
#endif

namespace sick_safetyscanners2_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RawMicroScanData_
{
  using Type = RawMicroScanData_<ContainerAllocator>;

  explicit RawMicroScanData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    derived_values(_init),
    general_system_state(_init),
    measurement_data(_init),
    intrusion_data(_init),
    application_data(_init)
  {
    (void)_init;
  }

  explicit RawMicroScanData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    derived_values(_alloc, _init),
    general_system_state(_alloc, _init),
    measurement_data(_alloc, _init),
    intrusion_data(_alloc, _init),
    application_data(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    sick_safetyscanners2_interfaces::msg::DataHeader_<ContainerAllocator>;
  _header_type header;
  using _derived_values_type =
    sick_safetyscanners2_interfaces::msg::DerivedValues_<ContainerAllocator>;
  _derived_values_type derived_values;
  using _general_system_state_type =
    sick_safetyscanners2_interfaces::msg::GeneralSystemState_<ContainerAllocator>;
  _general_system_state_type general_system_state;
  using _measurement_data_type =
    sick_safetyscanners2_interfaces::msg::MeasurementData_<ContainerAllocator>;
  _measurement_data_type measurement_data;
  using _intrusion_data_type =
    sick_safetyscanners2_interfaces::msg::IntrusionData_<ContainerAllocator>;
  _intrusion_data_type intrusion_data;
  using _application_data_type =
    sick_safetyscanners2_interfaces::msg::ApplicationData_<ContainerAllocator>;
  _application_data_type application_data;

  // setters for named parameter idiom
  Type & set__header(
    const sick_safetyscanners2_interfaces::msg::DataHeader_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__derived_values(
    const sick_safetyscanners2_interfaces::msg::DerivedValues_<ContainerAllocator> & _arg)
  {
    this->derived_values = _arg;
    return *this;
  }
  Type & set__general_system_state(
    const sick_safetyscanners2_interfaces::msg::GeneralSystemState_<ContainerAllocator> & _arg)
  {
    this->general_system_state = _arg;
    return *this;
  }
  Type & set__measurement_data(
    const sick_safetyscanners2_interfaces::msg::MeasurementData_<ContainerAllocator> & _arg)
  {
    this->measurement_data = _arg;
    return *this;
  }
  Type & set__intrusion_data(
    const sick_safetyscanners2_interfaces::msg::IntrusionData_<ContainerAllocator> & _arg)
  {
    this->intrusion_data = _arg;
    return *this;
  }
  Type & set__application_data(
    const sick_safetyscanners2_interfaces::msg::ApplicationData_<ContainerAllocator> & _arg)
  {
    this->application_data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sick_safetyscanners2_interfaces::msg::RawMicroScanData_<ContainerAllocator> *;
  using ConstRawPtr =
    const sick_safetyscanners2_interfaces::msg::RawMicroScanData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::RawMicroScanData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::RawMicroScanData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::msg::RawMicroScanData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::msg::RawMicroScanData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::msg::RawMicroScanData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::msg::RawMicroScanData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::msg::RawMicroScanData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::msg::RawMicroScanData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sick_safetyscanners2_interfaces__msg__RawMicroScanData
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::RawMicroScanData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sick_safetyscanners2_interfaces__msg__RawMicroScanData
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::RawMicroScanData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RawMicroScanData_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->derived_values != other.derived_values) {
      return false;
    }
    if (this->general_system_state != other.general_system_state) {
      return false;
    }
    if (this->measurement_data != other.measurement_data) {
      return false;
    }
    if (this->intrusion_data != other.intrusion_data) {
      return false;
    }
    if (this->application_data != other.application_data) {
      return false;
    }
    return true;
  }
  bool operator!=(const RawMicroScanData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RawMicroScanData_

// alias to use template instance with default allocator
using RawMicroScanData =
  sick_safetyscanners2_interfaces::msg::RawMicroScanData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__RAW_MICRO_SCAN_DATA__STRUCT_HPP_
