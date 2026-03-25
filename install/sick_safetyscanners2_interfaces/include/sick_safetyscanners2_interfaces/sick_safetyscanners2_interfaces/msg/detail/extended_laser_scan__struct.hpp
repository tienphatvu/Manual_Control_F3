// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/ExtendedLaserScan.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__EXTENDED_LASER_SCAN__STRUCT_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__EXTENDED_LASER_SCAN__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'laser_scan'
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__sick_safetyscanners2_interfaces__msg__ExtendedLaserScan __attribute__((deprecated))
#else
# define DEPRECATED__sick_safetyscanners2_interfaces__msg__ExtendedLaserScan __declspec(deprecated)
#endif

namespace sick_safetyscanners2_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ExtendedLaserScan_
{
  using Type = ExtendedLaserScan_<ContainerAllocator>;

  explicit ExtendedLaserScan_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : laser_scan(_init)
  {
    (void)_init;
  }

  explicit ExtendedLaserScan_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : laser_scan(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _laser_scan_type =
    sensor_msgs::msg::LaserScan_<ContainerAllocator>;
  _laser_scan_type laser_scan;
  using _reflektor_status_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _reflektor_status_type reflektor_status;
  using _reflektor_median_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _reflektor_median_type reflektor_median;
  using _intrusion_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _intrusion_type intrusion;

  // setters for named parameter idiom
  Type & set__laser_scan(
    const sensor_msgs::msg::LaserScan_<ContainerAllocator> & _arg)
  {
    this->laser_scan = _arg;
    return *this;
  }
  Type & set__reflektor_status(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->reflektor_status = _arg;
    return *this;
  }
  Type & set__reflektor_median(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->reflektor_median = _arg;
    return *this;
  }
  Type & set__intrusion(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->intrusion = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sick_safetyscanners2_interfaces::msg::ExtendedLaserScan_<ContainerAllocator> *;
  using ConstRawPtr =
    const sick_safetyscanners2_interfaces::msg::ExtendedLaserScan_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::ExtendedLaserScan_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::ExtendedLaserScan_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::msg::ExtendedLaserScan_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::msg::ExtendedLaserScan_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::msg::ExtendedLaserScan_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::msg::ExtendedLaserScan_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::msg::ExtendedLaserScan_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::msg::ExtendedLaserScan_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sick_safetyscanners2_interfaces__msg__ExtendedLaserScan
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::ExtendedLaserScan_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sick_safetyscanners2_interfaces__msg__ExtendedLaserScan
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::ExtendedLaserScan_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExtendedLaserScan_ & other) const
  {
    if (this->laser_scan != other.laser_scan) {
      return false;
    }
    if (this->reflektor_status != other.reflektor_status) {
      return false;
    }
    if (this->reflektor_median != other.reflektor_median) {
      return false;
    }
    if (this->intrusion != other.intrusion) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExtendedLaserScan_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExtendedLaserScan_

// alias to use template instance with default allocator
using ExtendedLaserScan =
  sick_safetyscanners2_interfaces::msg::ExtendedLaserScan_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__EXTENDED_LASER_SCAN__STRUCT_HPP_
