// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from sick_safetyscanners2_interfaces:msg/ScanPoint.idl
// generated code does not contain a copyright notice
#include "sick_safetyscanners2_interfaces/msg/detail/scan_point__rosidl_typesupport_fastrtps_cpp.hpp"
#include "sick_safetyscanners2_interfaces/msg/detail/scan_point__struct.hpp"

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
  const sick_safetyscanners2_interfaces::msg::ScanPoint & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: angle
  cdr << ros_message.angle;
  // Member: distance
  cdr << ros_message.distance;
  // Member: reflectivity
  cdr << ros_message.reflectivity;
  // Member: valid
  cdr << (ros_message.valid ? true : false);
  // Member: infinite
  cdr << (ros_message.infinite ? true : false);
  // Member: glare
  cdr << (ros_message.glare ? true : false);
  // Member: reflector
  cdr << (ros_message.reflector ? true : false);
  // Member: contamination
  cdr << (ros_message.contamination ? true : false);
  // Member: contamination_warning
  cdr << (ros_message.contamination_warning ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sick_safetyscanners2_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  sick_safetyscanners2_interfaces::msg::ScanPoint & ros_message)
{
  // Member: angle
  cdr >> ros_message.angle;

  // Member: distance
  cdr >> ros_message.distance;

  // Member: reflectivity
  cdr >> ros_message.reflectivity;

  // Member: valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.valid = tmp ? true : false;
  }

  // Member: infinite
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.infinite = tmp ? true : false;
  }

  // Member: glare
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.glare = tmp ? true : false;
  }

  // Member: reflector
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.reflector = tmp ? true : false;
  }

  // Member: contamination
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.contamination = tmp ? true : false;
  }

  // Member: contamination_warning
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.contamination_warning = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sick_safetyscanners2_interfaces
get_serialized_size(
  const sick_safetyscanners2_interfaces::msg::ScanPoint & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: angle
  {
    size_t item_size = sizeof(ros_message.angle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: distance
  {
    size_t item_size = sizeof(ros_message.distance);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: reflectivity
  {
    size_t item_size = sizeof(ros_message.reflectivity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: valid
  {
    size_t item_size = sizeof(ros_message.valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: infinite
  {
    size_t item_size = sizeof(ros_message.infinite);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: glare
  {
    size_t item_size = sizeof(ros_message.glare);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: reflector
  {
    size_t item_size = sizeof(ros_message.reflector);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: contamination
  {
    size_t item_size = sizeof(ros_message.contamination);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: contamination_warning
  {
    size_t item_size = sizeof(ros_message.contamination_warning);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sick_safetyscanners2_interfaces
max_serialized_size_ScanPoint(
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


  // Member: angle
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: distance
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: reflectivity
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: valid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: infinite
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: glare
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: reflector
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: contamination
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: contamination_warning
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
    using DataType = sick_safetyscanners2_interfaces::msg::ScanPoint;
    is_plain =
      (
      offsetof(DataType, contamination_warning) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _ScanPoint__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const sick_safetyscanners2_interfaces::msg::ScanPoint *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ScanPoint__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<sick_safetyscanners2_interfaces::msg::ScanPoint *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ScanPoint__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const sick_safetyscanners2_interfaces::msg::ScanPoint *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ScanPoint__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ScanPoint(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _ScanPoint__callbacks = {
  "sick_safetyscanners2_interfaces::msg",
  "ScanPoint",
  _ScanPoint__cdr_serialize,
  _ScanPoint__cdr_deserialize,
  _ScanPoint__get_serialized_size,
  _ScanPoint__max_serialized_size
};

static rosidl_message_type_support_t _ScanPoint__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ScanPoint__callbacks,
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
get_message_type_support_handle<sick_safetyscanners2_interfaces::msg::ScanPoint>()
{
  return &sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::_ScanPoint__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, sick_safetyscanners2_interfaces, msg, ScanPoint)() {
  return &sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::_ScanPoint__handle;
}

#ifdef __cplusplus
}
#endif
