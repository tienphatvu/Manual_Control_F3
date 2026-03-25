// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from sick_safetyscanners2_interfaces:msg/DataHeader.idl
// generated code does not contain a copyright notice
#include "sick_safetyscanners2_interfaces/msg/detail/data_header__rosidl_typesupport_fastrtps_cpp.hpp"
#include "sick_safetyscanners2_interfaces/msg/detail/data_header__struct.hpp"

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
  const sick_safetyscanners2_interfaces::msg::DataHeader & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: version_version
  cdr << ros_message.version_version;
  // Member: version_major_version
  cdr << ros_message.version_major_version;
  // Member: version_minor_version
  cdr << ros_message.version_minor_version;
  // Member: version_release
  cdr << ros_message.version_release;
  // Member: serial_number_of_device
  cdr << ros_message.serial_number_of_device;
  // Member: serial_number_of_channel_plug
  cdr << ros_message.serial_number_of_channel_plug;
  // Member: channel_number
  cdr << ros_message.channel_number;
  // Member: sequence_number
  cdr << ros_message.sequence_number;
  // Member: scan_number
  cdr << ros_message.scan_number;
  // Member: timestamp_date
  cdr << ros_message.timestamp_date;
  // Member: timestamp_time
  cdr << ros_message.timestamp_time;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sick_safetyscanners2_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  sick_safetyscanners2_interfaces::msg::DataHeader & ros_message)
{
  // Member: version_version
  cdr >> ros_message.version_version;

  // Member: version_major_version
  cdr >> ros_message.version_major_version;

  // Member: version_minor_version
  cdr >> ros_message.version_minor_version;

  // Member: version_release
  cdr >> ros_message.version_release;

  // Member: serial_number_of_device
  cdr >> ros_message.serial_number_of_device;

  // Member: serial_number_of_channel_plug
  cdr >> ros_message.serial_number_of_channel_plug;

  // Member: channel_number
  cdr >> ros_message.channel_number;

  // Member: sequence_number
  cdr >> ros_message.sequence_number;

  // Member: scan_number
  cdr >> ros_message.scan_number;

  // Member: timestamp_date
  cdr >> ros_message.timestamp_date;

  // Member: timestamp_time
  cdr >> ros_message.timestamp_time;

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sick_safetyscanners2_interfaces
get_serialized_size(
  const sick_safetyscanners2_interfaces::msg::DataHeader & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: version_version
  {
    size_t item_size = sizeof(ros_message.version_version);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: version_major_version
  {
    size_t item_size = sizeof(ros_message.version_major_version);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: version_minor_version
  {
    size_t item_size = sizeof(ros_message.version_minor_version);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: version_release
  {
    size_t item_size = sizeof(ros_message.version_release);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: serial_number_of_device
  {
    size_t item_size = sizeof(ros_message.serial_number_of_device);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: serial_number_of_channel_plug
  {
    size_t item_size = sizeof(ros_message.serial_number_of_channel_plug);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: channel_number
  {
    size_t item_size = sizeof(ros_message.channel_number);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: sequence_number
  {
    size_t item_size = sizeof(ros_message.sequence_number);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: scan_number
  {
    size_t item_size = sizeof(ros_message.scan_number);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: timestamp_date
  {
    size_t item_size = sizeof(ros_message.timestamp_date);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: timestamp_time
  {
    size_t item_size = sizeof(ros_message.timestamp_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sick_safetyscanners2_interfaces
max_serialized_size_DataHeader(
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


  // Member: version_version
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: version_major_version
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: version_minor_version
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: version_release
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: serial_number_of_device
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: serial_number_of_channel_plug
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: channel_number
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: sequence_number
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: scan_number
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: timestamp_date
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: timestamp_time
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
    using DataType = sick_safetyscanners2_interfaces::msg::DataHeader;
    is_plain =
      (
      offsetof(DataType, timestamp_time) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _DataHeader__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const sick_safetyscanners2_interfaces::msg::DataHeader *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _DataHeader__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<sick_safetyscanners2_interfaces::msg::DataHeader *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _DataHeader__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const sick_safetyscanners2_interfaces::msg::DataHeader *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _DataHeader__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_DataHeader(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _DataHeader__callbacks = {
  "sick_safetyscanners2_interfaces::msg",
  "DataHeader",
  _DataHeader__cdr_serialize,
  _DataHeader__cdr_deserialize,
  _DataHeader__get_serialized_size,
  _DataHeader__max_serialized_size
};

static rosidl_message_type_support_t _DataHeader__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_DataHeader__callbacks,
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
get_message_type_support_handle<sick_safetyscanners2_interfaces::msg::DataHeader>()
{
  return &sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::_DataHeader__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, sick_safetyscanners2_interfaces, msg, DataHeader)() {
  return &sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::_DataHeader__handle;
}

#ifdef __cplusplus
}
#endif
