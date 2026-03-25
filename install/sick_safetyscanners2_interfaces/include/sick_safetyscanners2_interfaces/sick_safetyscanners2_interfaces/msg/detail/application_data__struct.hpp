// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/ApplicationData.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_DATA__STRUCT_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'inputs'
#include "sick_safetyscanners2_interfaces/msg/detail/application_inputs__struct.hpp"
// Member 'outputs'
#include "sick_safetyscanners2_interfaces/msg/detail/application_outputs__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__sick_safetyscanners2_interfaces__msg__ApplicationData __attribute__((deprecated))
#else
# define DEPRECATED__sick_safetyscanners2_interfaces__msg__ApplicationData __declspec(deprecated)
#endif

namespace sick_safetyscanners2_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ApplicationData_
{
  using Type = ApplicationData_<ContainerAllocator>;

  explicit ApplicationData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : inputs(_init),
    outputs(_init)
  {
    (void)_init;
  }

  explicit ApplicationData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : inputs(_alloc, _init),
    outputs(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _inputs_type =
    sick_safetyscanners2_interfaces::msg::ApplicationInputs_<ContainerAllocator>;
  _inputs_type inputs;
  using _outputs_type =
    sick_safetyscanners2_interfaces::msg::ApplicationOutputs_<ContainerAllocator>;
  _outputs_type outputs;

  // setters for named parameter idiom
  Type & set__inputs(
    const sick_safetyscanners2_interfaces::msg::ApplicationInputs_<ContainerAllocator> & _arg)
  {
    this->inputs = _arg;
    return *this;
  }
  Type & set__outputs(
    const sick_safetyscanners2_interfaces::msg::ApplicationOutputs_<ContainerAllocator> & _arg)
  {
    this->outputs = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sick_safetyscanners2_interfaces::msg::ApplicationData_<ContainerAllocator> *;
  using ConstRawPtr =
    const sick_safetyscanners2_interfaces::msg::ApplicationData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::ApplicationData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::ApplicationData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::msg::ApplicationData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::msg::ApplicationData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sick_safetyscanners2_interfaces::msg::ApplicationData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sick_safetyscanners2_interfaces::msg::ApplicationData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::msg::ApplicationData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sick_safetyscanners2_interfaces::msg::ApplicationData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sick_safetyscanners2_interfaces__msg__ApplicationData
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::ApplicationData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sick_safetyscanners2_interfaces__msg__ApplicationData
    std::shared_ptr<sick_safetyscanners2_interfaces::msg::ApplicationData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ApplicationData_ & other) const
  {
    if (this->inputs != other.inputs) {
      return false;
    }
    if (this->outputs != other.outputs) {
      return false;
    }
    return true;
  }
  bool operator!=(const ApplicationData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ApplicationData_

// alias to use template instance with default allocator
using ApplicationData =
  sick_safetyscanners2_interfaces::msg::ApplicationData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sick_safetyscanners2_interfaces

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_DATA__STRUCT_HPP_
