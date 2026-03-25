// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/MeasurementData.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__MEASUREMENT_DATA__STRUCT_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__MEASUREMENT_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'scan_points'
#include "sick_safetyscanners2_interfaces/msg/detail/scan_point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__sick_safetyscanners2_interfaces__msg__MeasurementData __attribute__((deprecated))
#else
# define DEPRECATED__sick_safetyscanners2_interfaces__msg__MeasurementData __declspec(deprecated)
#endif

namespace sick_safetyscanners2_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MeasurementData_
{
  using Type = MeasurementData_<ContainerAllocator>;

  explicit MeasurementData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->number_of_beams = 0ul;
    }
  }

  explicit MeasurementData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->number_of_beams = 0ul;
    }
  }

  // field types and members
  using _number_of_beams_type =
    uint32_t;
  _number_of_beams_type number_of_beams;
  using _scan_points_type =
    std::vector<sick_safetyscanners2_interfaces::msg::ScanPoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sick_safetyscanners2_interfaces::msg::ScanPoint_<ContainerAllocator>>>;
  _scan_points_type scan_points;

  // setters for named parameter idiom
  Type & set__number_of_beams(
    const uint32_t & _arg)
  {
    this->number_of_beams = _arg;
    return *this;
  }
  Type & set__scan_points(
    const std::vector<sick_safetyscanners2_interfaces::msg::ScanPoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sick_safetyscanners2_interfaces::msg::ScanPoint_<ContainerAllocator>>> & _arg)
  {
    this->scan_points = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sick_safetyscanners2_interfaces::msg::MeasurementData_<ContainerAllocator> *;
  using ConstRawPtr =
    const sick_safetyscanners2_interfaces::msg::MeasurementData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::MeasurementData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::MeasurementData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::msg::MeasurementData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::msg::MeasurementData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::msg::MeasurementData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::msg::MeasurementData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::msg::MeasurementData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::msg::MeasurementData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sick_safetyscanners2_interfaces__msg__MeasurementData
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::MeasurementData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sick_safetyscanners2_interfaces__msg__MeasurementData
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::MeasurementData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MeasurementData_ & other) const
  {
    if (this->number_of_beams != other.number_of_beams) {
      return false;
    }
    if (this->scan_points != other.scan_points) {
      return false;
    }
    return true;
  }
  bool operator!=(const MeasurementData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MeasurementData_

// alias to use template instance with default allocator
using MeasurementData =
  sick_safetyscanners2_interfaces::msg::MeasurementData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__MEASUREMENT_DATA__STRUCT_HPP_
