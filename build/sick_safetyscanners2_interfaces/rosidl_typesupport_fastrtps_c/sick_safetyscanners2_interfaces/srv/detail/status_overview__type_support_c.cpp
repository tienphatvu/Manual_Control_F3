// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from sick_safetyscanners2_interfaces:srv/StatusOverview.idl
// generated code does not contain a copyright notice
#include "sick_safetyscanners2_interfaces/srv/detail/status_overview__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "sick_safetyscanners2_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "sick_safetyscanners2_interfaces/srv/detail/status_overview__struct.h"
#include "sick_safetyscanners2_interfaces/srv/detail/status_overview__functions.h"
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


using _StatusOverview_Request__ros_msg_type = sick_safetyscanners2_interfaces__srv__StatusOverview_Request;

static bool _StatusOverview_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _StatusOverview_Request__ros_msg_type * ros_message = static_cast<const _StatusOverview_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: structure_needs_at_least_one_member
  {
    cdr << ros_message->structure_needs_at_least_one_member;
  }

  return true;
}

static bool _StatusOverview_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _StatusOverview_Request__ros_msg_type * ros_message = static_cast<_StatusOverview_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: structure_needs_at_least_one_member
  {
    cdr >> ros_message->structure_needs_at_least_one_member;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sick_safetyscanners2_interfaces
size_t get_serialized_size_sick_safetyscanners2_interfaces__srv__StatusOverview_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _StatusOverview_Request__ros_msg_type * ros_message = static_cast<const _StatusOverview_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name structure_needs_at_least_one_member
  {
    size_t item_size = sizeof(ros_message->structure_needs_at_least_one_member);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _StatusOverview_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_sick_safetyscanners2_interfaces__srv__StatusOverview_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sick_safetyscanners2_interfaces
size_t max_serialized_size_sick_safetyscanners2_interfaces__srv__StatusOverview_Request(
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

  // member: structure_needs_at_least_one_member
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
    using DataType = sick_safetyscanners2_interfaces__srv__StatusOverview_Request;
    is_plain =
      (
      offsetof(DataType, structure_needs_at_least_one_member) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _StatusOverview_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_sick_safetyscanners2_interfaces__srv__StatusOverview_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_StatusOverview_Request = {
  "sick_safetyscanners2_interfaces::srv",
  "StatusOverview_Request",
  _StatusOverview_Request__cdr_serialize,
  _StatusOverview_Request__cdr_deserialize,
  _StatusOverview_Request__get_serialized_size,
  _StatusOverview_Request__max_serialized_size
};

static rosidl_message_type_support_t _StatusOverview_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_StatusOverview_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sick_safetyscanners2_interfaces, srv, StatusOverview_Request)() {
  return &_StatusOverview_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "sick_safetyscanners2_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "sick_safetyscanners2_interfaces/srv/detail/status_overview__struct.h"
// already included above
// #include "sick_safetyscanners2_interfaces/srv/detail/status_overview__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

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

#include "rosidl_runtime_c/string.h"  // current_time, error_info_time, version_c_version
#include "rosidl_runtime_c/string_functions.h"  // current_time, error_info_time, version_c_version

// forward declare type support functions


using _StatusOverview_Response__ros_msg_type = sick_safetyscanners2_interfaces__srv__StatusOverview_Response;

static bool _StatusOverview_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _StatusOverview_Response__ros_msg_type * ros_message = static_cast<const _StatusOverview_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: version_c_version
  {
    const rosidl_runtime_c__String * str = &ros_message->version_c_version;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: version_major_version_number
  {
    cdr << ros_message->version_major_version_number;
  }

  // Field name: version_minor_version_number
  {
    cdr << ros_message->version_minor_version_number;
  }

  // Field name: version_release_number
  {
    cdr << ros_message->version_release_number;
  }

  // Field name: device_state
  {
    cdr << ros_message->device_state;
  }

  // Field name: config_state
  {
    cdr << ros_message->config_state;
  }

  // Field name: application_state
  {
    cdr << ros_message->application_state;
  }

  // Field name: current_time_power_on_count
  {
    cdr << ros_message->current_time_power_on_count;
  }

  // Field name: current_time
  {
    const rosidl_runtime_c__String * str = &ros_message->current_time;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: current_time_time
  {
    cdr << ros_message->current_time_time;
  }

  // Field name: current_time_date
  {
    cdr << ros_message->current_time_date;
  }

  // Field name: error_info_code
  {
    cdr << ros_message->error_info_code;
  }

  // Field name: error_info_time
  {
    const rosidl_runtime_c__String * str = &ros_message->error_info_time;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: error_info_time_time
  {
    cdr << ros_message->error_info_time_time;
  }

  // Field name: error_info_time_date
  {
    cdr << ros_message->error_info_time_date;
  }

  return true;
}

static bool _StatusOverview_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _StatusOverview_Response__ros_msg_type * ros_message = static_cast<_StatusOverview_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: version_c_version
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->version_c_version.data) {
      rosidl_runtime_c__String__init(&ros_message->version_c_version);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->version_c_version,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'version_c_version'\n");
      return false;
    }
  }

  // Field name: version_major_version_number
  {
    cdr >> ros_message->version_major_version_number;
  }

  // Field name: version_minor_version_number
  {
    cdr >> ros_message->version_minor_version_number;
  }

  // Field name: version_release_number
  {
    cdr >> ros_message->version_release_number;
  }

  // Field name: device_state
  {
    cdr >> ros_message->device_state;
  }

  // Field name: config_state
  {
    cdr >> ros_message->config_state;
  }

  // Field name: application_state
  {
    cdr >> ros_message->application_state;
  }

  // Field name: current_time_power_on_count
  {
    cdr >> ros_message->current_time_power_on_count;
  }

  // Field name: current_time
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->current_time.data) {
      rosidl_runtime_c__String__init(&ros_message->current_time);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->current_time,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'current_time'\n");
      return false;
    }
  }

  // Field name: current_time_time
  {
    cdr >> ros_message->current_time_time;
  }

  // Field name: current_time_date
  {
    cdr >> ros_message->current_time_date;
  }

  // Field name: error_info_code
  {
    cdr >> ros_message->error_info_code;
  }

  // Field name: error_info_time
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->error_info_time.data) {
      rosidl_runtime_c__String__init(&ros_message->error_info_time);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->error_info_time,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'error_info_time'\n");
      return false;
    }
  }

  // Field name: error_info_time_time
  {
    cdr >> ros_message->error_info_time_time;
  }

  // Field name: error_info_time_date
  {
    cdr >> ros_message->error_info_time_date;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sick_safetyscanners2_interfaces
size_t get_serialized_size_sick_safetyscanners2_interfaces__srv__StatusOverview_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _StatusOverview_Response__ros_msg_type * ros_message = static_cast<const _StatusOverview_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name version_c_version
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->version_c_version.size + 1);
  // field.name version_major_version_number
  {
    size_t item_size = sizeof(ros_message->version_major_version_number);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name version_minor_version_number
  {
    size_t item_size = sizeof(ros_message->version_minor_version_number);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name version_release_number
  {
    size_t item_size = sizeof(ros_message->version_release_number);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name device_state
  {
    size_t item_size = sizeof(ros_message->device_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name config_state
  {
    size_t item_size = sizeof(ros_message->config_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name application_state
  {
    size_t item_size = sizeof(ros_message->application_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current_time_power_on_count
  {
    size_t item_size = sizeof(ros_message->current_time_power_on_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current_time
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->current_time.size + 1);
  // field.name current_time_time
  {
    size_t item_size = sizeof(ros_message->current_time_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current_time_date
  {
    size_t item_size = sizeof(ros_message->current_time_date);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name error_info_code
  {
    size_t item_size = sizeof(ros_message->error_info_code);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name error_info_time
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->error_info_time.size + 1);
  // field.name error_info_time_time
  {
    size_t item_size = sizeof(ros_message->error_info_time_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name error_info_time_date
  {
    size_t item_size = sizeof(ros_message->error_info_time_date);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _StatusOverview_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_sick_safetyscanners2_interfaces__srv__StatusOverview_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sick_safetyscanners2_interfaces
size_t max_serialized_size_sick_safetyscanners2_interfaces__srv__StatusOverview_Response(
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

  // member: version_c_version
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: version_major_version_number
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: version_minor_version_number
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: version_release_number
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: device_state
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: config_state
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: application_state
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: current_time_power_on_count
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: current_time
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: current_time_time
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: current_time_date
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: error_info_code
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: error_info_time
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: error_info_time_time
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: error_info_time_date
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = sick_safetyscanners2_interfaces__srv__StatusOverview_Response;
    is_plain =
      (
      offsetof(DataType, error_info_time_date) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _StatusOverview_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_sick_safetyscanners2_interfaces__srv__StatusOverview_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_StatusOverview_Response = {
  "sick_safetyscanners2_interfaces::srv",
  "StatusOverview_Response",
  _StatusOverview_Response__cdr_serialize,
  _StatusOverview_Response__cdr_deserialize,
  _StatusOverview_Response__get_serialized_size,
  _StatusOverview_Response__max_serialized_size
};

static rosidl_message_type_support_t _StatusOverview_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_StatusOverview_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sick_safetyscanners2_interfaces, srv, StatusOverview_Response)() {
  return &_StatusOverview_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "sick_safetyscanners2_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "sick_safetyscanners2_interfaces/srv/status_overview.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t StatusOverview__callbacks = {
  "sick_safetyscanners2_interfaces::srv",
  "StatusOverview",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sick_safetyscanners2_interfaces, srv, StatusOverview_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sick_safetyscanners2_interfaces, srv, StatusOverview_Response)(),
};

static rosidl_service_type_support_t StatusOverview__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &StatusOverview__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sick_safetyscanners2_interfaces, srv, StatusOverview)() {
  return &StatusOverview__handle;
}

#if defined(__cplusplus)
}
#endif
