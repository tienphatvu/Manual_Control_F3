// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from sick_safetyscanners2_interfaces:msg/Field.idl
// generated code does not contain a copyright notice
#include "sick_safetyscanners2_interfaces/msg/detail/field__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "sick_safetyscanners2_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "sick_safetyscanners2_interfaces/msg/detail/field__struct.h"
#include "sick_safetyscanners2_interfaces/msg/detail/field__functions.h"
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

#include "rosidl_runtime_c/primitives_sequence.h"  // ranges
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // ranges

// forward declare type support functions


using _Field__ros_msg_type = sick_safetyscanners2_interfaces__msg__Field;

static bool _Field__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Field__ros_msg_type * ros_message = static_cast<const _Field__ros_msg_type *>(untyped_ros_message);
  // Field name: ranges
  {
    size_t size = ros_message->ranges.size;
    auto array_ptr = ros_message->ranges.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: start_angle
  {
    cdr << ros_message->start_angle;
  }

  // Field name: angular_resolution
  {
    cdr << ros_message->angular_resolution;
  }

  // Field name: protective_field
  {
    cdr << (ros_message->protective_field ? true : false);
  }

  return true;
}

static bool _Field__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Field__ros_msg_type * ros_message = static_cast<_Field__ros_msg_type *>(untyped_ros_message);
  // Field name: ranges
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

    if (ros_message->ranges.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->ranges);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->ranges, size)) {
      fprintf(stderr, "failed to create array for field 'ranges'");
      return false;
    }
    auto array_ptr = ros_message->ranges.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: start_angle
  {
    cdr >> ros_message->start_angle;
  }

  // Field name: angular_resolution
  {
    cdr >> ros_message->angular_resolution;
  }

  // Field name: protective_field
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->protective_field = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sick_safetyscanners2_interfaces
size_t get_serialized_size_sick_safetyscanners2_interfaces__msg__Field(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Field__ros_msg_type * ros_message = static_cast<const _Field__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name ranges
  {
    size_t array_size = ros_message->ranges.size;
    auto array_ptr = ros_message->ranges.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name start_angle
  {
    size_t item_size = sizeof(ros_message->start_angle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name angular_resolution
  {
    size_t item_size = sizeof(ros_message->angular_resolution);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name protective_field
  {
    size_t item_size = sizeof(ros_message->protective_field);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Field__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_sick_safetyscanners2_interfaces__msg__Field(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sick_safetyscanners2_interfaces
size_t max_serialized_size_sick_safetyscanners2_interfaces__msg__Field(
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

  // member: ranges
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
  // member: start_angle
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: angular_resolution
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: protective_field
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
    using DataType = sick_safetyscanners2_interfaces__msg__Field;
    is_plain =
      (
      offsetof(DataType, protective_field) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Field__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_sick_safetyscanners2_interfaces__msg__Field(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Field = {
  "sick_safetyscanners2_interfaces::msg",
  "Field",
  _Field__cdr_serialize,
  _Field__cdr_deserialize,
  _Field__get_serialized_size,
  _Field__max_serialized_size
};

static rosidl_message_type_support_t _Field__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Field,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sick_safetyscanners2_interfaces, msg, Field)() {
  return &_Field__type_support;
}

#if defined(__cplusplus)
}
#endif
