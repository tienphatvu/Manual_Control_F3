// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from sick_safetyscanners2_interfaces:msg/RawMicroScanData.idl
// generated code does not contain a copyright notice
#include "sick_safetyscanners2_interfaces/msg/detail/raw_micro_scan_data__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "sick_safetyscanners2_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "sick_safetyscanners2_interfaces/msg/detail/raw_micro_scan_data__struct.h"
#include "sick_safetyscanners2_interfaces/msg/detail/raw_micro_scan_data__functions.h"
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

#include "sick_safetyscanners2_interfaces/msg/detail/application_data__functions.h"  // application_data
#include "sick_safetyscanners2_interfaces/msg/detail/data_header__functions.h"  // header
#include "sick_safetyscanners2_interfaces/msg/detail/derived_values__functions.h"  // derived_values
#include "sick_safetyscanners2_interfaces/msg/detail/general_system_state__functions.h"  // general_system_state
#include "sick_safetyscanners2_interfaces/msg/detail/intrusion_data__functions.h"  // intrusion_data
#include "sick_safetyscanners2_interfaces/msg/detail/measurement_data__functions.h"  // measurement_data

// forward declare type support functions
size_t get_serialized_size_sick_safetyscanners2_interfaces__msg__ApplicationData(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_sick_safetyscanners2_interfaces__msg__ApplicationData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sick_safetyscanners2_interfaces, msg, ApplicationData)();
size_t get_serialized_size_sick_safetyscanners2_interfaces__msg__DataHeader(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_sick_safetyscanners2_interfaces__msg__DataHeader(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sick_safetyscanners2_interfaces, msg, DataHeader)();
size_t get_serialized_size_sick_safetyscanners2_interfaces__msg__DerivedValues(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_sick_safetyscanners2_interfaces__msg__DerivedValues(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sick_safetyscanners2_interfaces, msg, DerivedValues)();
size_t get_serialized_size_sick_safetyscanners2_interfaces__msg__GeneralSystemState(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_sick_safetyscanners2_interfaces__msg__GeneralSystemState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sick_safetyscanners2_interfaces, msg, GeneralSystemState)();
size_t get_serialized_size_sick_safetyscanners2_interfaces__msg__IntrusionData(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_sick_safetyscanners2_interfaces__msg__IntrusionData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sick_safetyscanners2_interfaces, msg, IntrusionData)();
size_t get_serialized_size_sick_safetyscanners2_interfaces__msg__MeasurementData(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_sick_safetyscanners2_interfaces__msg__MeasurementData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sick_safetyscanners2_interfaces, msg, MeasurementData)();


using _RawMicroScanData__ros_msg_type = sick_safetyscanners2_interfaces__msg__RawMicroScanData;

static bool _RawMicroScanData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _RawMicroScanData__ros_msg_type * ros_message = static_cast<const _RawMicroScanData__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sick_safetyscanners2_interfaces, msg, DataHeader
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: derived_values
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sick_safetyscanners2_interfaces, msg, DerivedValues
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->derived_values, cdr))
    {
      return false;
    }
  }

  // Field name: general_system_state
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sick_safetyscanners2_interfaces, msg, GeneralSystemState
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->general_system_state, cdr))
    {
      return false;
    }
  }

  // Field name: measurement_data
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sick_safetyscanners2_interfaces, msg, MeasurementData
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->measurement_data, cdr))
    {
      return false;
    }
  }

  // Field name: intrusion_data
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sick_safetyscanners2_interfaces, msg, IntrusionData
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->intrusion_data, cdr))
    {
      return false;
    }
  }

  // Field name: application_data
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sick_safetyscanners2_interfaces, msg, ApplicationData
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->application_data, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _RawMicroScanData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _RawMicroScanData__ros_msg_type * ros_message = static_cast<_RawMicroScanData__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sick_safetyscanners2_interfaces, msg, DataHeader
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: derived_values
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sick_safetyscanners2_interfaces, msg, DerivedValues
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->derived_values))
    {
      return false;
    }
  }

  // Field name: general_system_state
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sick_safetyscanners2_interfaces, msg, GeneralSystemState
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->general_system_state))
    {
      return false;
    }
  }

  // Field name: measurement_data
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sick_safetyscanners2_interfaces, msg, MeasurementData
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->measurement_data))
    {
      return false;
    }
  }

  // Field name: intrusion_data
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sick_safetyscanners2_interfaces, msg, IntrusionData
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->intrusion_data))
    {
      return false;
    }
  }

  // Field name: application_data
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sick_safetyscanners2_interfaces, msg, ApplicationData
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->application_data))
    {
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sick_safetyscanners2_interfaces
size_t get_serialized_size_sick_safetyscanners2_interfaces__msg__RawMicroScanData(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _RawMicroScanData__ros_msg_type * ros_message = static_cast<const _RawMicroScanData__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_sick_safetyscanners2_interfaces__msg__DataHeader(
    &(ros_message->header), current_alignment);
  // field.name derived_values

  current_alignment += get_serialized_size_sick_safetyscanners2_interfaces__msg__DerivedValues(
    &(ros_message->derived_values), current_alignment);
  // field.name general_system_state

  current_alignment += get_serialized_size_sick_safetyscanners2_interfaces__msg__GeneralSystemState(
    &(ros_message->general_system_state), current_alignment);
  // field.name measurement_data

  current_alignment += get_serialized_size_sick_safetyscanners2_interfaces__msg__MeasurementData(
    &(ros_message->measurement_data), current_alignment);
  // field.name intrusion_data

  current_alignment += get_serialized_size_sick_safetyscanners2_interfaces__msg__IntrusionData(
    &(ros_message->intrusion_data), current_alignment);
  // field.name application_data

  current_alignment += get_serialized_size_sick_safetyscanners2_interfaces__msg__ApplicationData(
    &(ros_message->application_data), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _RawMicroScanData__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_sick_safetyscanners2_interfaces__msg__RawMicroScanData(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sick_safetyscanners2_interfaces
size_t max_serialized_size_sick_safetyscanners2_interfaces__msg__RawMicroScanData(
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

  // member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_sick_safetyscanners2_interfaces__msg__DataHeader(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: derived_values
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_sick_safetyscanners2_interfaces__msg__DerivedValues(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: general_system_state
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_sick_safetyscanners2_interfaces__msg__GeneralSystemState(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: measurement_data
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_sick_safetyscanners2_interfaces__msg__MeasurementData(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: intrusion_data
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_sick_safetyscanners2_interfaces__msg__IntrusionData(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: application_data
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_sick_safetyscanners2_interfaces__msg__ApplicationData(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = sick_safetyscanners2_interfaces__msg__RawMicroScanData;
    is_plain =
      (
      offsetof(DataType, application_data) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _RawMicroScanData__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_sick_safetyscanners2_interfaces__msg__RawMicroScanData(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_RawMicroScanData = {
  "sick_safetyscanners2_interfaces::msg",
  "RawMicroScanData",
  _RawMicroScanData__cdr_serialize,
  _RawMicroScanData__cdr_deserialize,
  _RawMicroScanData__get_serialized_size,
  _RawMicroScanData__max_serialized_size
};

static rosidl_message_type_support_t _RawMicroScanData__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_RawMicroScanData,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sick_safetyscanners2_interfaces, msg, RawMicroScanData)() {
  return &_RawMicroScanData__type_support;
}

#if defined(__cplusplus)
}
#endif
