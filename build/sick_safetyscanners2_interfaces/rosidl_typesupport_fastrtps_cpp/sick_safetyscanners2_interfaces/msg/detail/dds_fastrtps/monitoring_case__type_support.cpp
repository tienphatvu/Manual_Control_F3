// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from sick_safetyscanners2_interfaces:msg/MonitoringCase.idl
// generated code does not contain a copyright notice
#include "sick_safetyscanners2_interfaces/msg/detail/monitoring_case__rosidl_typesupport_fastrtps_cpp.hpp"
#include "sick_safetyscanners2_interfaces/msg/detail/monitoring_case__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace sick_safetyscanners2_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sick_safetyscanners2_interfaces
cdr_serialize(
  const sick_safetyscanners2_interfaces::msg::MonitoringCase & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: monitoring_case_number
  cdr << ros_message.monitoring_case_number;
  // Member: fields
  {
    cdr << ros_message.fields;
  }
  // Member: fields_valid
  {
    cdr << ros_message.fields_valid;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sick_safetyscanners2_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  sick_safetyscanners2_interfaces::msg::MonitoringCase & ros_message)
{
  // Member: monitoring_case_number
  cdr >> ros_message.monitoring_case_number;

  // Member: fields
  {
    cdr >> ros_message.fields;
  }

  // Member: fields_valid
  {
    cdr >> ros_message.fields_valid;
  }

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sick_safetyscanners2_interfaces
get_serialized_size(
  const sick_safetyscanners2_interfaces::msg::MonitoringCase & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: monitoring_case_number
  {
    size_t item_size = sizeof(ros_message.monitoring_case_number);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fields
  {
    size_t array_size = ros_message.fields.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.fields[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fields_valid
  {
    size_t array_size = ros_message.fields_valid.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.fields_valid[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sick_safetyscanners2_interfaces
max_serialized_size_MonitoringCase(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: monitoring_case_number
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: fields
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: fields_valid
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = sick_safetyscanners2_interfaces::msg::MonitoringCase;
    is_plain =
      (
      offsetof(DataType, fields_valid) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _MonitoringCase__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const sick_safetyscanners2_interfaces::msg::MonitoringCase *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _MonitoringCase__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<sick_safetyscanners2_interfaces::msg::MonitoringCase *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _MonitoringCase__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const sick_safetyscanners2_interfaces::msg::MonitoringCase *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _MonitoringCase__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_MonitoringCase(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _MonitoringCase__callbacks = {
  "sick_safetyscanners2_interfaces::msg",
  "MonitoringCase",
  _MonitoringCase__cdr_serialize,
  _MonitoringCase__cdr_deserialize,
  _MonitoringCase__get_serialized_size,
  _MonitoringCase__max_serialized_size
};

static rosidl_message_type_support_t _MonitoringCase__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_MonitoringCase__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace sick_safetyscanners2_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_sick_safetyscanners2_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<sick_safetyscanners2_interfaces::msg::MonitoringCase>()
{
  return &sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::_MonitoringCase__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, sick_safetyscanners2_interfaces, msg, MonitoringCase)() {
  return &sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::_MonitoringCase__handle;
}

#ifdef __cplusplus
}
#endif
