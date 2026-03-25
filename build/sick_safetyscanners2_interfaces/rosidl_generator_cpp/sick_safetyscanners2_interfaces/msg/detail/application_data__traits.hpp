// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sick_safetyscanners2_interfaces:msg/ApplicationData.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_DATA__TRAITS_HPP_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sick_safetyscanners2_interfaces/msg/detail/application_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'inputs'
#include "sick_safetyscanners2_interfaces/msg/detail/application_inputs__traits.hpp"
// Member 'outputs'
#include "sick_safetyscanners2_interfaces/msg/detail/application_outputs__traits.hpp"

namespace sick_safetyscanners2_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ApplicationData & msg,
  std::ostream & out)
{
  out << "{";
  // member: inputs
  {
    out << "inputs: ";
    to_flow_style_yaml(msg.inputs, out);
    out << ", ";
  }

  // member: outputs
  {
    out << "outputs: ";
    to_flow_style_yaml(msg.outputs, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ApplicationData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: inputs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "inputs:\n";
    to_block_style_yaml(msg.inputs, out, indentation + 2);
  }

  // member: outputs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "outputs:\n";
    to_block_style_yaml(msg.outputs, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ApplicationData & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace sick_safetyscanners2_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use sick_safetyscanners2_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const sick_safetyscanners2_interfaces::msg::ApplicationData & msg,
  std::ostream & out, size_t indentation = 0)
{
  sick_safetyscanners2_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sick_safetyscanners2_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const sick_safetyscanners2_interfaces::msg::ApplicationData & msg)
{
  return sick_safetyscanners2_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sick_safetyscanners2_interfaces::msg::ApplicationData>()
{
  return "sick_safetyscanners2_interfaces::msg::ApplicationData";
}

template<>
inline const char * name<sick_safetyscanners2_interfaces::msg::ApplicationData>()
{
  return "sick_safetyscanners2_interfaces/msg/ApplicationData";
}

template<>
struct has_fixed_size<sick_safetyscanners2_interfaces::msg::ApplicationData>
  : std::integral_constant<bool, has_fixed_size<sick_safetyscanners2_interfaces::msg::ApplicationInputs>::value && has_fixed_size<sick_safetyscanners2_interfaces::msg::ApplicationOutputs>::value> {};

template<>
struct has_bounded_size<sick_safetyscanners2_interfaces::msg::ApplicationData>
  : std::integral_constant<bool, has_bounded_size<sick_safetyscanners2_interfaces::msg::ApplicationInputs>::value && has_bounded_size<sick_safetyscanners2_interfaces::msg::ApplicationOutputs>::value> {};

template<>
struct is_message<sick_safetyscanners2_interfaces::msg::ApplicationData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__APPLICATION_DATA__TRAITS_HPP_
