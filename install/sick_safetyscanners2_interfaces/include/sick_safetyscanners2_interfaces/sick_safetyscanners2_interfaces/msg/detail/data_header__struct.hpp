// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/DataHeader.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__DATA_HEADER__STRUCT_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__DATA_HEADER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__sick_safetyscanners2_interfaces__msg__DataHeader __attribute__((deprecated))
#else
# define DEPRECATED__sick_safetyscanners2_interfaces__msg__DataHeader __declspec(deprecated)
#endif

namespace sick_safetyscanners2_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DataHeader_
{
  using Type = DataHeader_<ContainerAllocator>;

  explicit DataHeader_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->version_version = 0;
      this->version_major_version = 0;
      this->version_minor_version = 0;
      this->version_release = 0;
      this->serial_number_of_device = 0ul;
      this->serial_number_of_channel_plug = 0ul;
      this->channel_number = 0;
      this->sequence_number = 0ul;
      this->scan_number = 0ul;
      this->timestamp_date = 0;
      this->timestamp_time = 0ul;
    }
  }

  explicit DataHeader_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->version_version = 0;
      this->version_major_version = 0;
      this->version_minor_version = 0;
      this->version_release = 0;
      this->serial_number_of_device = 0ul;
      this->serial_number_of_channel_plug = 0ul;
      this->channel_number = 0;
      this->sequence_number = 0ul;
      this->scan_number = 0ul;
      this->timestamp_date = 0;
      this->timestamp_time = 0ul;
    }
  }

  // field types and members
  using _version_version_type =
    uint8_t;
  _version_version_type version_version;
  using _version_major_version_type =
    uint8_t;
  _version_major_version_type version_major_version;
  using _version_minor_version_type =
    uint8_t;
  _version_minor_version_type version_minor_version;
  using _version_release_type =
    uint8_t;
  _version_release_type version_release;
  using _serial_number_of_device_type =
    uint32_t;
  _serial_number_of_device_type serial_number_of_device;
  using _serial_number_of_channel_plug_type =
    uint32_t;
  _serial_number_of_channel_plug_type serial_number_of_channel_plug;
  using _channel_number_type =
    uint8_t;
  _channel_number_type channel_number;
  using _sequence_number_type =
    uint32_t;
  _sequence_number_type sequence_number;
  using _scan_number_type =
    uint32_t;
  _scan_number_type scan_number;
  using _timestamp_date_type =
    uint16_t;
  _timestamp_date_type timestamp_date;
  using _timestamp_time_type =
    uint32_t;
  _timestamp_time_type timestamp_time;

  // setters for named parameter idiom
  Type & set__version_version(
    const uint8_t & _arg)
  {
    this->version_version = _arg;
    return *this;
  }
  Type & set__version_major_version(
    const uint8_t & _arg)
  {
    this->version_major_version = _arg;
    return *this;
  }
  Type & set__version_minor_version(
    const uint8_t & _arg)
  {
    this->version_minor_version = _arg;
    return *this;
  }
  Type & set__version_release(
    const uint8_t & _arg)
  {
    this->version_release = _arg;
    return *this;
  }
  Type & set__serial_number_of_device(
    const uint32_t & _arg)
  {
    this->serial_number_of_device = _arg;
    return *this;
  }
  Type & set__serial_number_of_channel_plug(
    const uint32_t & _arg)
  {
    this->serial_number_of_channel_plug = _arg;
    return *this;
  }
  Type & set__channel_number(
    const uint8_t & _arg)
  {
    this->channel_number = _arg;
    return *this;
  }
  Type & set__sequence_number(
    const uint32_t & _arg)
  {
    this->sequence_number = _arg;
    return *this;
  }
  Type & set__scan_number(
    const uint32_t & _arg)
  {
    this->scan_number = _arg;
    return *this;
  }
  Type & set__timestamp_date(
    const uint16_t & _arg)
  {
    this->timestamp_date = _arg;
    return *this;
  }
  Type & set__timestamp_time(
    const uint32_t & _arg)
  {
    this->timestamp_time = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sick_safetyscanners2_interfaces::msg::DataHeader_<ContainerAllocator> *;
  using ConstRawPtr =
    const sick_safetyscanners2_interfaces::msg::DataHeader_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::DataHeader_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::DataHeader_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::msg::DataHeader_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::msg::DataHeader_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::msg::DataHeader_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::msg::DataHeader_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::msg::DataHeader_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::msg::DataHeader_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sick_safetyscanners2_interfaces__msg__DataHeader
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::DataHeader_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sick_safetyscanners2_interfaces__msg__DataHeader
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::DataHeader_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DataHeader_ & other) const
  {
    if (this->version_version != other.version_version) {
      return false;
    }
    if (this->version_major_version != other.version_major_version) {
      return false;
    }
    if (this->version_minor_version != other.version_minor_version) {
      return false;
    }
    if (this->version_release != other.version_release) {
      return false;
    }
    if (this->serial_number_of_device != other.serial_number_of_device) {
      return false;
    }
    if (this->serial_number_of_channel_plug != other.serial_number_of_channel_plug) {
      return false;
    }
    if (this->channel_number != other.channel_number) {
      return false;
    }
    if (this->sequence_number != other.sequence_number) {
      return false;
    }
    if (this->scan_number != other.scan_number) {
      return false;
    }
    if (this->timestamp_date != other.timestamp_date) {
      return false;
    }
    if (this->timestamp_time != other.timestamp_time) {
      return false;
    }
    return true;
  }
  bool operator!=(const DataHeader_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DataHeader_

// alias to use template instance with default allocator
using DataHeader =
  sick_safetyscanners2_interfaces::msg::DataHeader_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__DATA_HEADER__STRUCT_HPP_
