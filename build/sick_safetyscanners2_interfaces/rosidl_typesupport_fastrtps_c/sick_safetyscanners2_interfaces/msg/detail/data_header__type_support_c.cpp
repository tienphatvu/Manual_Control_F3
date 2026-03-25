// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from sick_safetyscanners2_interfaces:msg/DataHeader.idl
// generated code does not contain a copyright notice
#include "sick_safetyscanners2_interfaces/msg/detail/data_header__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "sick_safetyscanners2_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "sick_safetyscanners2_interfaces/msg/detail/data_header__struct.h"
#include "sick_safetyscanners2_interfaces/msg/detail/data_header__functions.h"
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


// forward declare type support functions


using _DataHeader__ros_msg_type = sick_safetyscanners2_interfaces__msg__DataHeader;

static bool _DataHeader__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _DataHeader__ros_msg_type * ros_message = static_cast<const _DataHeader__ros_msg_type *>(untyped_ros_message);
  // Field name: version_version
  {
    cdr << ros_message->version_version;
  }

  // Field name: version_major_version
  {
    cdr << ros_message->version_major_version;
  }

  // Field name: version_minor_version
  {
    cdr << ros_message->version_minor_version;
  }

  // Field name: version_release
  {
    cdr << ros_message->version_release;
  }

  // Field name: serial_number_of_device
  {
    cdr << ros_message->serial_number_of_device;
  }

  // Field name: serial_number_of_channel_plug
  {
    cdr << ros_message->serial_number_of_channel_plug;
  }

  // Field name: channel_number
  {
    cdr << ros_message->channel_number;
  }

  // Field name: sequence_number
  {
    cdr << ros_message->sequence_number;
  }

  // Field name: scan_number
  {
    cdr << ros_message->scan_number;
  }

  // Field name: timestamp_date
  {
    cdr << ros_message->timestamp_date;
  }

  // Field name: timestamp_time
  {
    cdr << ros_message->timestamp_time;
  }

  return true;
}

static bool _DataHeader__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _DataHeader__ros_msg_type * ros_message = static_cast<_DataHeader__ros_msg_type *>(untyped_ros_message);
  // Field name: version_version
  {
    cdr >> ros_message->version_version;
  }

  // Field name: version_major_version
  {
    cdr >> ros_message->version_major_version;
  }

  // Field name: version_minor_version
  {
    cdr >> ros_message->version_minor_version;
  }

  // Field name: version_release
  {
    cdr >> ros_message->version_release;
  }

  // Field name: serial_number_of_device
  {
    cdr >> ros_message->serial_number_of_device;
  }

  // Field name: serial_number_of_channel_plug
  {
    cdr >> ros_message->serial_number_of_channel_plug;
  }

  // Field name: channel_number
  {
    cdr >> ros_message->channel_number;
  }

  // Field name: sequence_number
  {
    cdr >> ros_message->sequence_number;
  }

  // Field name: scan_number
  {
    cdr >> ros_message->scan_number;
  }

  // Field name: timestamp_date
  {
    cdr >> ros_message->timestamp_date;
  }

  // Field name: timestamp_time
  {
    cdr >> ros_message->timestamp_time;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sick_safetyscanners2_interfaces
size_t get_serialized_size_sick_safetyscanners2_interfaces__msg__DataHeader(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _DataHeader__ros_msg_type * ros_message = static_cast<const _DataHeader__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name version_version
  {
    size_t item_size = sizeof(ros_message->version_version);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name version_major_version
  {
    size_t item_size = sizeof(ros_message->version_major_version);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name version_minor_version
  {
    size_t item_size = sizeof(ros_message->version_minor_version);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name version_release
  {
    size_t item_size = sizeof(ros_message->version_release);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name serial_number_of_device
  {
    size_t item_size = sizeof(ros_message->serial_number_of_device);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name serial_number_of_channel_plug
  {
    size_t item_size = sizeof(ros_message->serial_number_of_channel_plug);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name channel_number
  {
    size_t item_size = sizeof(ros_message->channel_number);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name sequence_number
  {
    size_t item_size = sizeof(ros_message->sequence_number);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name scan_number
  {
    size_t item_size = sizeof(ros_message->scan_number);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name timestamp_date
  {
    size_t item_size = sizeof(ros_message->timestamp_date);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name timestamp_time
  {
    size_t item_size = sizeof(ros_message->timestamp_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _DataHeader__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_sick_safetyscanners2_interfaces__msg__DataHeader(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sick_safetyscanners2_interfaces
size_t max_serialized_size_sick_safetyscanners2_interfaces__msg__DataHeader(
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

  // member: version_version
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: version_major_version
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: version_minor_version
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: version_release
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: serial_number_of_device
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: serial_number_of_channel_plug
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: channel_number
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: sequence_number
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: scan_number
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: timestamp_date
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: timestamp_time
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = sick_safetyscanners2_interfaces__msg__DataHeader;
    is_plain =
      (
      offsetof(DataType, timestamp_time) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _DataHeader__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_sick_safetyscanners2_interfaces__msg__DataHeader(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_DataHeader = {
  "sick_safetyscanners2_interfaces::msg",
  "DataHeader",
  _DataHeader__cdr_serialize,
  _DataHeader__cdr_deserialize,
  _DataHeader__get_serialized_size,
  _DataHeader__max_serialized_size
};

static rosidl_message_type_support_t _DataHeader__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_DataHeader,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sick_safetyscanners2_interfaces, msg, DataHeader)() {
  return &_DataHeader__type_support;
}

#if defined(__cplusplus)
}
#endif
