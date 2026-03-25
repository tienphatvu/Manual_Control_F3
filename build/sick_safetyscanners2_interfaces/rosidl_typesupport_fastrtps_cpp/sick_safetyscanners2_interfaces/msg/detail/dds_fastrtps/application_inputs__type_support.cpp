// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from sick_safetyscanners2_interfaces:msg/ApplicationInputs.idl
// generated code does not contain a copyright notice
#include "sick_safetyscanners2_interfaces/msg/detail/application_inputs__rosidl_typesupport_fastrtps_cpp.hpp"
#include "sick_safetyscanners2_interfaces/msg/detail/application_inputs__struct.hpp"

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
  const sick_safetyscanners2_interfaces::msg::ApplicationInputs & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: unsafe_inputs_input_sources
  {
    cdr << ros_message.unsafe_inputs_input_sources;
  }
  // Member: unsafe_inputs_flags
  {
    cdr << ros_message.unsafe_inputs_flags;
  }
  // Member: monitoring_case_number_inputs
  {
    cdr << ros_message.monitoring_case_number_inputs;
  }
  // Member: monitoring_case_number_inputs_flags
  {
    cdr << ros_message.monitoring_case_number_inputs_flags;
  }
  // Member: linear_velocity_inputs_velocity_0
  cdr << ros_message.linear_velocity_inputs_velocity_0;
  // Member: linear_velocity_inputs_velocity_0_valid
  cdr << (ros_message.linear_velocity_inputs_velocity_0_valid ? true : false);
  // Member: linear_velocity_inputs_velocity_0_transmitted_safely
  cdr << (ros_message.linear_velocity_inputs_velocity_0_transmitted_safely ? true : false);
  // Member: linear_velocity_inputs_velocity_1
  cdr << ros_message.linear_velocity_inputs_velocity_1;
  // Member: linear_velocity_inputs_velocity_1_valid
  cdr << (ros_message.linear_velocity_inputs_velocity_1_valid ? true : false);
  // Member: linear_velocity_inputs_velocity_1_transmitted_safely
  cdr << (ros_message.linear_velocity_inputs_velocity_1_transmitted_safely ? true : false);
  // Member: sleep_mode_input
  cdr << ros_message.sleep_mode_input;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sick_safetyscanners2_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  sick_safetyscanners2_interfaces::msg::ApplicationInputs & ros_message)
{
  // Member: unsafe_inputs_input_sources
  {
    cdr >> ros_message.unsafe_inputs_input_sources;
  }

  // Member: unsafe_inputs_flags
  {
    cdr >> ros_message.unsafe_inputs_flags;
  }

  // Member: monitoring_case_number_inputs
  {
    cdr >> ros_message.monitoring_case_number_inputs;
  }

  // Member: monitoring_case_number_inputs_flags
  {
    cdr >> ros_message.monitoring_case_number_inputs_flags;
  }

  // Member: linear_velocity_inputs_velocity_0
  cdr >> ros_message.linear_velocity_inputs_velocity_0;

  // Member: linear_velocity_inputs_velocity_0_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.linear_velocity_inputs_velocity_0_valid = tmp ? true : false;
  }

  // Member: linear_velocity_inputs_velocity_0_transmitted_safely
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.linear_velocity_inputs_velocity_0_transmitted_safely = tmp ? true : false;
  }

  // Member: linear_velocity_inputs_velocity_1
  cdr >> ros_message.linear_velocity_inputs_velocity_1;

  // Member: linear_velocity_inputs_velocity_1_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.linear_velocity_inputs_velocity_1_valid = tmp ? true : false;
  }

  // Member: linear_velocity_inputs_velocity_1_transmitted_safely
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.linear_velocity_inputs_velocity_1_transmitted_safely = tmp ? true : false;
  }

  // Member: sleep_mode_input
  cdr >> ros_message.sleep_mode_input;

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sick_safetyscanners2_interfaces
get_serialized_size(
  const sick_safetyscanners2_interfaces::msg::ApplicationInputs & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: unsafe_inputs_input_sources
  {
    size_t array_size = ros_message.unsafe_inputs_input_sources.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.unsafe_inputs_input_sources[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: unsafe_inputs_flags
  {
    size_t array_size = ros_message.unsafe_inputs_flags.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.unsafe_inputs_flags[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: monitoring_case_number_inputs
  {
    size_t array_size = ros_message.monitoring_case_number_inputs.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.monitoring_case_number_inputs[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: monitoring_case_number_inputs_flags
  {
    size_t array_size = ros_message.monitoring_case_number_inputs_flags.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.monitoring_case_number_inputs_flags[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: linear_velocity_inputs_velocity_0
  {
    size_t item_size = sizeof(ros_message.linear_velocity_inputs_velocity_0);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: linear_velocity_inputs_velocity_0_valid
  {
    size_t item_size = sizeof(ros_message.linear_velocity_inputs_velocity_0_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: linear_velocity_inputs_velocity_0_transmitted_safely
  {
    size_t item_size = sizeof(ros_message.linear_velocity_inputs_velocity_0_transmitted_safely);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: linear_velocity_inputs_velocity_1
  {
    size_t item_size = sizeof(ros_message.linear_velocity_inputs_velocity_1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: linear_velocity_inputs_velocity_1_valid
  {
    size_t item_size = sizeof(ros_message.linear_velocity_inputs_velocity_1_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: linear_velocity_inputs_velocity_1_transmitted_safely
  {
    size_t item_size = sizeof(ros_message.linear_velocity_inputs_velocity_1_transmitted_safely);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: sleep_mode_input
  {
    size_t item_size = sizeof(ros_message.sleep_mode_input);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sick_safetyscanners2_interfaces
max_serialized_size_ApplicationInputs(
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


  // Member: unsafe_inputs_input_sources
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: unsafe_inputs_flags
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: monitoring_case_number_inputs
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: monitoring_case_number_inputs_flags
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: linear_velocity_inputs_velocity_0
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: linear_velocity_inputs_velocity_0_valid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: linear_velocity_inputs_velocity_0_transmitted_safely
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: linear_velocity_inputs_velocity_1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: linear_velocity_inputs_velocity_1_valid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: linear_velocity_inputs_velocity_1_transmitted_safely
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: sleep_mode_input
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = sick_safetyscanners2_interfaces::msg::ApplicationInputs;
    is_plain =
      (
      offsetof(DataType, sleep_mode_input) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _ApplicationInputs__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const sick_safetyscanners2_interfaces::msg::ApplicationInputs *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ApplicationInputs__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<sick_safetyscanners2_interfaces::msg::ApplicationInputs *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ApplicationInputs__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const sick_safetyscanners2_interfaces::msg::ApplicationInputs *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ApplicationInputs__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ApplicationInputs(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _ApplicationInputs__callbacks = {
  "sick_safetyscanners2_interfaces::msg",
  "ApplicationInputs",
  _ApplicationInputs__cdr_serialize,
  _ApplicationInputs__cdr_deserialize,
  _ApplicationInputs__get_serialized_size,
  _ApplicationInputs__max_serialized_size
};

static rosidl_message_type_support_t _ApplicationInputs__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ApplicationInputs__callbacks,
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
get_message_type_support_handle<sick_safetyscanners2_interfaces::msg::ApplicationInputs>()
{
  return &sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::_ApplicationInputs__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, sick_safetyscanners2_interfaces, msg, ApplicationInputs)() {
  return &sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::_ApplicationInputs__handle;
}

#ifdef __cplusplus
}
#endif
