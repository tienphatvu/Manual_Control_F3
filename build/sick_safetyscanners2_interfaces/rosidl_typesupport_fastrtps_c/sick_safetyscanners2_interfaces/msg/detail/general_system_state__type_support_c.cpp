// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from sick_safetyscanners2_interfaces:msg/GeneralSystemState.idl
// generated code does not contain a copyright notice
#include "sick_safetyscanners2_interfaces/msg/detail/general_system_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "sick_safetyscanners2_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "sick_safetyscanners2_interfaces/msg/detail/general_system_state__struct.h"
#include "sick_safetyscanners2_interfaces/msg/detail/general_system_state__functions.h"
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

#include "rosidl_runtime_c/primitives_sequence.h"  // non_safe_cut_off_path, reset_required_cut_off_path, safe_cut_off_path
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // non_safe_cut_off_path, reset_required_cut_off_path, safe_cut_off_path

// forward declare type support functions


using _GeneralSystemState__ros_msg_type = sick_safetyscanners2_interfaces__msg__GeneralSystemState;

static bool _GeneralSystemState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GeneralSystemState__ros_msg_type * ros_message = static_cast<const _GeneralSystemState__ros_msg_type *>(untyped_ros_message);
  // Field name: run_mode_active
  {
    cdr << (ros_message->run_mode_active ? true : false);
  }

  // Field name: standby_mode_active
  {
    cdr << (ros_message->standby_mode_active ? true : false);
  }

  // Field name: contamination_warning
  {
    cdr << (ros_message->contamination_warning ? true : false);
  }

  // Field name: contamination_error
  {
    cdr << (ros_message->contamination_error ? true : false);
  }

  // Field name: reference_contour_status
  {
    cdr << (ros_message->reference_contour_status ? true : false);
  }

  // Field name: manipulation_status
  {
    cdr << (ros_message->manipulation_status ? true : false);
  }

  // Field name: safe_cut_off_path
  {
    size_t size = ros_message->safe_cut_off_path.size;
    auto array_ptr = ros_message->safe_cut_off_path.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: non_safe_cut_off_path
  {
    size_t size = ros_message->non_safe_cut_off_path.size;
    auto array_ptr = ros_message->non_safe_cut_off_path.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: reset_required_cut_off_path
  {
    size_t size = ros_message->reset_required_cut_off_path.size;
    auto array_ptr = ros_message->reset_required_cut_off_path.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: current_monitoring_case_no_table_1
  {
    cdr << ros_message->current_monitoring_case_no_table_1;
  }

  // Field name: current_monitoring_case_no_table_2
  {
    cdr << ros_message->current_monitoring_case_no_table_2;
  }

  // Field name: current_monitoring_case_no_table_3
  {
    cdr << ros_message->current_monitoring_case_no_table_3;
  }

  // Field name: current_monitoring_case_no_table_4
  {
    cdr << ros_message->current_monitoring_case_no_table_4;
  }

  // Field name: application_error
  {
    cdr << (ros_message->application_error ? true : false);
  }

  // Field name: device_error
  {
    cdr << (ros_message->device_error ? true : false);
  }

  return true;
}

static bool _GeneralSystemState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GeneralSystemState__ros_msg_type * ros_message = static_cast<_GeneralSystemState__ros_msg_type *>(untyped_ros_message);
  // Field name: run_mode_active
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->run_mode_active = tmp ? true : false;
  }

  // Field name: standby_mode_active
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->standby_mode_active = tmp ? true : false;
  }

  // Field name: contamination_warning
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->contamination_warning = tmp ? true : false;
  }

  // Field name: contamination_error
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->contamination_error = tmp ? true : false;
  }

  // Field name: reference_contour_status
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->reference_contour_status = tmp ? true : false;
  }

  // Field name: manipulation_status
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->manipulation_status = tmp ? true : false;
  }

  // Field name: safe_cut_off_path
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

    if (ros_message->safe_cut_off_path.data) {
      rosidl_runtime_c__boolean__Sequence__fini(&ros_message->safe_cut_off_path);
    }
    if (!rosidl_runtime_c__boolean__Sequence__init(&ros_message->safe_cut_off_path, size)) {
      fprintf(stderr, "failed to create array for field 'safe_cut_off_path'");
      return false;
    }
    auto array_ptr = ros_message->safe_cut_off_path.data;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  // Field name: non_safe_cut_off_path
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

    if (ros_message->non_safe_cut_off_path.data) {
      rosidl_runtime_c__boolean__Sequence__fini(&ros_message->non_safe_cut_off_path);
    }
    if (!rosidl_runtime_c__boolean__Sequence__init(&ros_message->non_safe_cut_off_path, size)) {
      fprintf(stderr, "failed to create array for field 'non_safe_cut_off_path'");
      return false;
    }
    auto array_ptr = ros_message->non_safe_cut_off_path.data;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  // Field name: reset_required_cut_off_path
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

    if (ros_message->reset_required_cut_off_path.data) {
      rosidl_runtime_c__boolean__Sequence__fini(&ros_message->reset_required_cut_off_path);
    }
    if (!rosidl_runtime_c__boolean__Sequence__init(&ros_message->reset_required_cut_off_path, size)) {
      fprintf(stderr, "failed to create array for field 'reset_required_cut_off_path'");
      return false;
    }
    auto array_ptr = ros_message->reset_required_cut_off_path.data;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  // Field name: current_monitoring_case_no_table_1
  {
    cdr >> ros_message->current_monitoring_case_no_table_1;
  }

  // Field name: current_monitoring_case_no_table_2
  {
    cdr >> ros_message->current_monitoring_case_no_table_2;
  }

  // Field name: current_monitoring_case_no_table_3
  {
    cdr >> ros_message->current_monitoring_case_no_table_3;
  }

  // Field name: current_monitoring_case_no_table_4
  {
    cdr >> ros_message->current_monitoring_case_no_table_4;
  }

  // Field name: application_error
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->application_error = tmp ? true : false;
  }

  // Field name: device_error
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->device_error = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sick_safetyscanners2_interfaces
size_t get_serialized_size_sick_safetyscanners2_interfaces__msg__GeneralSystemState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GeneralSystemState__ros_msg_type * ros_message = static_cast<const _GeneralSystemState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name run_mode_active
  {
    size_t item_size = sizeof(ros_message->run_mode_active);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name standby_mode_active
  {
    size_t item_size = sizeof(ros_message->standby_mode_active);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name contamination_warning
  {
    size_t item_size = sizeof(ros_message->contamination_warning);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name contamination_error
  {
    size_t item_size = sizeof(ros_message->contamination_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name reference_contour_status
  {
    size_t item_size = sizeof(ros_message->reference_contour_status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name manipulation_status
  {
    size_t item_size = sizeof(ros_message->manipulation_status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name safe_cut_off_path
  {
    size_t array_size = ros_message->safe_cut_off_path.size;
    auto array_ptr = ros_message->safe_cut_off_path.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name non_safe_cut_off_path
  {
    size_t array_size = ros_message->non_safe_cut_off_path.size;
    auto array_ptr = ros_message->non_safe_cut_off_path.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name reset_required_cut_off_path
  {
    size_t array_size = ros_message->reset_required_cut_off_path.size;
    auto array_ptr = ros_message->reset_required_cut_off_path.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current_monitoring_case_no_table_1
  {
    size_t item_size = sizeof(ros_message->current_monitoring_case_no_table_1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current_monitoring_case_no_table_2
  {
    size_t item_size = sizeof(ros_message->current_monitoring_case_no_table_2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current_monitoring_case_no_table_3
  {
    size_t item_size = sizeof(ros_message->current_monitoring_case_no_table_3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current_monitoring_case_no_table_4
  {
    size_t item_size = sizeof(ros_message->current_monitoring_case_no_table_4);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name application_error
  {
    size_t item_size = sizeof(ros_message->application_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name device_error
  {
    size_t item_size = sizeof(ros_message->device_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _GeneralSystemState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_sick_safetyscanners2_interfaces__msg__GeneralSystemState(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sick_safetyscanners2_interfaces
size_t max_serialized_size_sick_safetyscanners2_interfaces__msg__GeneralSystemState(
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

  // member: run_mode_active
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: standby_mode_active
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: contamination_warning
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: contamination_error
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: reference_contour_status
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: manipulation_status
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: safe_cut_off_path
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: non_safe_cut_off_path
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: reset_required_cut_off_path
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: current_monitoring_case_no_table_1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: current_monitoring_case_no_table_2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: current_monitoring_case_no_table_3
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: current_monitoring_case_no_table_4
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: application_error
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: device_error
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
    using DataType = sick_safetyscanners2_interfaces__msg__GeneralSystemState;
    is_plain =
      (
      offsetof(DataType, device_error) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GeneralSystemState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_sick_safetyscanners2_interfaces__msg__GeneralSystemState(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GeneralSystemState = {
  "sick_safetyscanners2_interfaces::msg",
  "GeneralSystemState",
  _GeneralSystemState__cdr_serialize,
  _GeneralSystemState__cdr_deserialize,
  _GeneralSystemState__get_serialized_size,
  _GeneralSystemState__max_serialized_size
};

static rosidl_message_type_support_t _GeneralSystemState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GeneralSystemState,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sick_safetyscanners2_interfaces, msg, GeneralSystemState)() {
  return &_GeneralSystemState__type_support;
}

#if defined(__cplusplus)
}
#endif
