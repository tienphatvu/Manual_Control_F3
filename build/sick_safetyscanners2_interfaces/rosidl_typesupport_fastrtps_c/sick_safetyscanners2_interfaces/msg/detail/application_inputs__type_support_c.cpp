// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from sick_safetyscanners2_interfaces:msg/ApplicationInputs.idl
// generated code does not contain a copyright notice
#include "sick_safetyscanners2_interfaces/msg/detail/application_inputs__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "sick_safetyscanners2_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "sick_safetyscanners2_interfaces/msg/detail/application_inputs__struct.h"
#include "sick_safetyscanners2_interfaces/msg/detail/application_inputs__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/primitives_sequence.h"  // monitoring_case_number_inputs, monitoring_case_number_inputs_flags, unsafe_inputs_flags, unsafe_inputs_input_sources
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // monitoring_case_number_inputs, monitoring_case_number_inputs_flags, unsafe_inputs_flags, unsafe_inputs_input_sources

// forward declare type support functions


using _ApplicationInputs__ros_msg_type = sick_safetyscanners2_interfaces__msg__ApplicationInputs;

static bool _ApplicationInputs__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ApplicationInputs__ros_msg_type * ros_message = static_cast<const _ApplicationInputs__ros_msg_type *>(untyped_ros_message);
  // Field name: unsafe_inputs_input_sources
  {
    size_t size = ros_message->unsafe_inputs_input_sources.size;
    auto array_ptr = ros_message->unsafe_inputs_input_sources.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: unsafe_inputs_flags
  {
    size_t size = ros_message->unsafe_inputs_flags.size;
    auto array_ptr = ros_message->unsafe_inputs_flags.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: monitoring_case_number_inputs
  {
    size_t size = ros_message->monitoring_case_number_inputs.size;
    auto array_ptr = ros_message->monitoring_case_number_inputs.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: monitoring_case_number_inputs_flags
  {
    size_t size = ros_message->monitoring_case_number_inputs_flags.size;
    auto array_ptr = ros_message->monitoring_case_number_inputs_flags.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: linear_velocity_inputs_velocity_0
  {
    cdr << ros_message->linear_velocity_inputs_velocity_0;
  }

  // Field name: linear_velocity_inputs_velocity_0_valid
  {
    cdr << (ros_message->linear_velocity_inputs_velocity_0_valid ? true : false);
  }

  // Field name: linear_velocity_inputs_velocity_0_transmitted_safely
  {
    cdr << (ros_message->linear_velocity_inputs_velocity_0_transmitted_safely ? true : false);
  }

  // Field name: linear_velocity_inputs_velocity_1
  {
    cdr << ros_message->linear_velocity_inputs_velocity_1;
  }

  // Field name: linear_velocity_inputs_velocity_1_valid
  {
    cdr << (ros_message->linear_velocity_inputs_velocity_1_valid ? true : false);
  }

  // Field name: linear_velocity_inputs_velocity_1_transmitted_safely
  {
    cdr << (ros_message->linear_velocity_inputs_velocity_1_transmitted_safely ? true : false);
  }

  // Field name: sleep_mode_input
  {
    cdr << ros_message->sleep_mode_input;
  }

  return true;
}

static bool _ApplicationInputs__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ApplicationInputs__ros_msg_type * ros_message = static_cast<_ApplicationInputs__ros_msg_type *>(untyped_ros_message);
  // Field name: unsafe_inputs_input_sources
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.getState();
    bool correct_size = cdr.jump(size);
    cdr.setState(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->unsafe_inputs_input_sources.data) {
      rosidl_runtime_c__boolean__Sequence__fini(&ros_message->unsafe_inputs_input_sources);
    }
    if (!rosidl_runtime_c__boolean__Sequence__init(&ros_message->unsafe_inputs_input_sources, size)) {
      fprintf(stderr, "failed to create array for field 'unsafe_inputs_input_sources'");
      return false;
    }
    auto array_ptr = ros_message->unsafe_inputs_input_sources.data;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  // Field name: unsafe_inputs_flags
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.getState();
    bool correct_size = cdr.jump(size);
    cdr.setState(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->unsafe_inputs_flags.data) {
      rosidl_runtime_c__boolean__Sequence__fini(&ros_message->unsafe_inputs_flags);
    }
    if (!rosidl_runtime_c__boolean__Sequence__init(&ros_message->unsafe_inputs_flags, size)) {
      fprintf(stderr, "failed to create array for field 'unsafe_inputs_flags'");
      return false;
    }
    auto array_ptr = ros_message->unsafe_inputs_flags.data;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  // Field name: monitoring_case_number_inputs
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.getState();
    bool correct_size = cdr.jump(size);
    cdr.setState(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->monitoring_case_number_inputs.data) {
      rosidl_runtime_c__uint16__Sequence__fini(&ros_message->monitoring_case_number_inputs);
    }
    if (!rosidl_runtime_c__uint16__Sequence__init(&ros_message->monitoring_case_number_inputs, size)) {
      fprintf(stderr, "failed to create array for field 'monitoring_case_number_inputs'");
      return false;
    }
    auto array_ptr = ros_message->monitoring_case_number_inputs.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: monitoring_case_number_inputs_flags
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.getState();
    bool correct_size = cdr.jump(size);
    cdr.setState(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->monitoring_case_number_inputs_flags.data) {
      rosidl_runtime_c__boolean__Sequence__fini(&ros_message->monitoring_case_number_inputs_flags);
    }
    if (!rosidl_runtime_c__boolean__Sequence__init(&ros_message->monitoring_case_number_inputs_flags, size)) {
      fprintf(stderr, "failed to create array for field 'monitoring_case_number_inputs_flags'");
      return false;
    }
    auto array_ptr = ros_message->monitoring_case_number_inputs_flags.data;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  // Field name: linear_velocity_inputs_velocity_0
  {
    cdr >> ros_message->linear_velocity_inputs_velocity_0;
  }

  // Field name: linear_velocity_inputs_velocity_0_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->linear_velocity_inputs_velocity_0_valid = tmp ? true : false;
  }

  // Field name: linear_velocity_inputs_velocity_0_transmitted_safely
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->linear_velocity_inputs_velocity_0_transmitted_safely = tmp ? true : false;
  }

  // Field name: linear_velocity_inputs_velocity_1
  {
    cdr >> ros_message->linear_velocity_inputs_velocity_1;
  }

  // Field name: linear_velocity_inputs_velocity_1_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->linear_velocity_inputs_velocity_1_valid = tmp ? true : false;
  }

  // Field name: linear_velocity_inputs_velocity_1_transmitted_safely
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->linear_velocity_inputs_velocity_1_transmitted_safely = tmp ? true : false;
  }

  // Field name: sleep_mode_input
  {
    cdr >> ros_message->sleep_mode_input;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sick_safetyscanners2_interfaces
size_t get_serialized_size_sick_safetyscanners2_interfaces__msg__ApplicationInputs(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ApplicationInputs__ros_msg_type * ros_message = static_cast<const _ApplicationInputs__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name unsafe_inputs_input_sources
  {
    size_t array_size = ros_message->unsafe_inputs_input_sources.size;
    auto array_ptr = ros_message->unsafe_inputs_input_sources.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name unsafe_inputs_flags
  {
    size_t array_size = ros_message->unsafe_inputs_flags.size;
    auto array_ptr = ros_message->unsafe_inputs_flags.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name monitoring_case_number_inputs
  {
    size_t array_size = ros_message->monitoring_case_number_inputs.size;
    auto array_ptr = ros_message->monitoring_case_number_inputs.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name monitoring_case_number_inputs_flags
  {
    size_t array_size = ros_message->monitoring_case_number_inputs_flags.size;
    auto array_ptr = ros_message->monitoring_case_number_inputs_flags.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name linear_velocity_inputs_velocity_0
  {
    size_t item_size = sizeof(ros_message->linear_velocity_inputs_velocity_0);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name linear_velocity_inputs_velocity_0_valid
  {
    size_t item_size = sizeof(ros_message->linear_velocity_inputs_velocity_0_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name linear_velocity_inputs_velocity_0_transmitted_safely
  {
    size_t item_size = sizeof(ros_message->linear_velocity_inputs_velocity_0_transmitted_safely);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name linear_velocity_inputs_velocity_1
  {
    size_t item_size = sizeof(ros_message->linear_velocity_inputs_velocity_1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name linear_velocity_inputs_velocity_1_valid
  {
    size_t item_size = sizeof(ros_message->linear_velocity_inputs_velocity_1_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name linear_velocity_inputs_velocity_1_transmitted_safely
  {
    size_t item_size = sizeof(ros_message->linear_velocity_inputs_velocity_1_transmitted_safely);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name sleep_mode_input
  {
    size_t item_size = sizeof(ros_message->sleep_mode_input);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _ApplicationInputs__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_sick_safetyscanners2_interfaces__msg__ApplicationInputs(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sick_safetyscanners2_interfaces
size_t max_serialized_size_sick_safetyscanners2_interfaces__msg__ApplicationInputs(
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

  // member: unsafe_inputs_input_sources
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: unsafe_inputs_flags
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: monitoring_case_number_inputs
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
  // member: monitoring_case_number_inputs_flags
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: linear_velocity_inputs_velocity_0
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: linear_velocity_inputs_velocity_0_valid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: linear_velocity_inputs_velocity_0_transmitted_safely
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: linear_velocity_inputs_velocity_1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: linear_velocity_inputs_velocity_1_valid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: linear_velocity_inputs_velocity_1_transmitted_safely
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: sleep_mode_input
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
    using DataType = sick_safetyscanners2_interfaces__msg__ApplicationInputs;
    is_plain =
      (
      offsetof(DataType, sleep_mode_input) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _ApplicationInputs__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_sick_safetyscanners2_interfaces__msg__ApplicationInputs(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ApplicationInputs = {
  "sick_safetyscanners2_interfaces::msg",
  "ApplicationInputs",
  _ApplicationInputs__cdr_serialize,
  _ApplicationInputs__cdr_deserialize,
  _ApplicationInputs__get_serialized_size,
  _ApplicationInputs__max_serialized_size
};

static rosidl_message_type_support_t _ApplicationInputs__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ApplicationInputs,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sick_safetyscanners2_interfaces, msg, ApplicationInputs)() {
  return &_ApplicationInputs__type_support;
}

#if defined(__cplusplus)
}
#endif
