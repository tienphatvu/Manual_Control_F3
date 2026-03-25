// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from sick_safetyscanners2_interfaces:msg/ApplicationOutputs.idl
// generated code does not contain a copyright notice
#include "sick_safetyscanners2_interfaces/msg/detail/application_outputs__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "sick_safetyscanners2_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "sick_safetyscanners2_interfaces/msg/detail/application_outputs__struct.h"
#include "sick_safetyscanners2_interfaces/msg/detail/application_outputs__functions.h"
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

#include "rosidl_runtime_c/primitives_sequence.h"  // evaluation_path_outputs_eval_out, evaluation_path_outputs_is_safe, evaluation_path_outputs_is_valid, monitoring_case_number_outputs, monitoring_case_number_outputs_flags, resulting_velocity, resulting_velocity_flags
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // evaluation_path_outputs_eval_out, evaluation_path_outputs_is_safe, evaluation_path_outputs_is_valid, monitoring_case_number_outputs, monitoring_case_number_outputs_flags, resulting_velocity, resulting_velocity_flags

// forward declare type support functions


using _ApplicationOutputs__ros_msg_type = sick_safetyscanners2_interfaces__msg__ApplicationOutputs;

static bool _ApplicationOutputs__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ApplicationOutputs__ros_msg_type * ros_message = static_cast<const _ApplicationOutputs__ros_msg_type *>(untyped_ros_message);
  // Field name: evaluation_path_outputs_eval_out
  {
    size_t size = ros_message->evaluation_path_outputs_eval_out.size;
    auto array_ptr = ros_message->evaluation_path_outputs_eval_out.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: evaluation_path_outputs_is_safe
  {
    size_t size = ros_message->evaluation_path_outputs_is_safe.size;
    auto array_ptr = ros_message->evaluation_path_outputs_is_safe.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: evaluation_path_outputs_is_valid
  {
    size_t size = ros_message->evaluation_path_outputs_is_valid.size;
    auto array_ptr = ros_message->evaluation_path_outputs_is_valid.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: monitoring_case_number_outputs
  {
    size_t size = ros_message->monitoring_case_number_outputs.size;
    auto array_ptr = ros_message->monitoring_case_number_outputs.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: monitoring_case_number_outputs_flags
  {
    size_t size = ros_message->monitoring_case_number_outputs_flags.size;
    auto array_ptr = ros_message->monitoring_case_number_outputs_flags.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: sleep_mode_output
  {
    cdr << ros_message->sleep_mode_output;
  }

  // Field name: sleep_mode_output_valid
  {
    cdr << (ros_message->sleep_mode_output_valid ? true : false);
  }

  // Field name: error_flag_contamination_warning
  {
    cdr << (ros_message->error_flag_contamination_warning ? true : false);
  }

  // Field name: error_flag_contamination_error
  {
    cdr << (ros_message->error_flag_contamination_error ? true : false);
  }

  // Field name: error_flag_manipulation_error
  {
    cdr << (ros_message->error_flag_manipulation_error ? true : false);
  }

  // Field name: error_flag_glare
  {
    cdr << (ros_message->error_flag_glare ? true : false);
  }

  // Field name: error_flag_reference_contour_intruded
  {
    cdr << (ros_message->error_flag_reference_contour_intruded ? true : false);
  }

  // Field name: error_flag_critical_error
  {
    cdr << (ros_message->error_flag_critical_error ? true : false);
  }

  // Field name: error_flags_are_valid
  {
    cdr << (ros_message->error_flags_are_valid ? true : false);
  }

  // Field name: linear_velocity_outputs_velocity_0
  {
    cdr << ros_message->linear_velocity_outputs_velocity_0;
  }

  // Field name: linear_velocity_outputs_velocity_0_valid
  {
    cdr << (ros_message->linear_velocity_outputs_velocity_0_valid ? true : false);
  }

  // Field name: linear_velocity_outputs_velocity_0_transmitted_safely
  {
    cdr << (ros_message->linear_velocity_outputs_velocity_0_transmitted_safely ? true : false);
  }

  // Field name: linear_velocity_outputs_velocity_1
  {
    cdr << ros_message->linear_velocity_outputs_velocity_1;
  }

  // Field name: linear_velocity_outputs_velocity_1_valid
  {
    cdr << (ros_message->linear_velocity_outputs_velocity_1_valid ? true : false);
  }

  // Field name: linear_velocity_outputs_velocity_1_transmitted_safely
  {
    cdr << (ros_message->linear_velocity_outputs_velocity_1_transmitted_safely ? true : false);
  }

  // Field name: resulting_velocity
  {
    size_t size = ros_message->resulting_velocity.size;
    auto array_ptr = ros_message->resulting_velocity.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: resulting_velocity_flags
  {
    size_t size = ros_message->resulting_velocity_flags.size;
    auto array_ptr = ros_message->resulting_velocity_flags.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _ApplicationOutputs__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ApplicationOutputs__ros_msg_type * ros_message = static_cast<_ApplicationOutputs__ros_msg_type *>(untyped_ros_message);
  // Field name: evaluation_path_outputs_eval_out
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

    if (ros_message->evaluation_path_outputs_eval_out.data) {
      rosidl_runtime_c__boolean__Sequence__fini(&ros_message->evaluation_path_outputs_eval_out);
    }
    if (!rosidl_runtime_c__boolean__Sequence__init(&ros_message->evaluation_path_outputs_eval_out, size)) {
      fprintf(stderr, "failed to create array for field 'evaluation_path_outputs_eval_out'");
      return false;
    }
    auto array_ptr = ros_message->evaluation_path_outputs_eval_out.data;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  // Field name: evaluation_path_outputs_is_safe
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

    if (ros_message->evaluation_path_outputs_is_safe.data) {
      rosidl_runtime_c__boolean__Sequence__fini(&ros_message->evaluation_path_outputs_is_safe);
    }
    if (!rosidl_runtime_c__boolean__Sequence__init(&ros_message->evaluation_path_outputs_is_safe, size)) {
      fprintf(stderr, "failed to create array for field 'evaluation_path_outputs_is_safe'");
      return false;
    }
    auto array_ptr = ros_message->evaluation_path_outputs_is_safe.data;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  // Field name: evaluation_path_outputs_is_valid
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

    if (ros_message->evaluation_path_outputs_is_valid.data) {
      rosidl_runtime_c__boolean__Sequence__fini(&ros_message->evaluation_path_outputs_is_valid);
    }
    if (!rosidl_runtime_c__boolean__Sequence__init(&ros_message->evaluation_path_outputs_is_valid, size)) {
      fprintf(stderr, "failed to create array for field 'evaluation_path_outputs_is_valid'");
      return false;
    }
    auto array_ptr = ros_message->evaluation_path_outputs_is_valid.data;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  // Field name: monitoring_case_number_outputs
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

    if (ros_message->monitoring_case_number_outputs.data) {
      rosidl_runtime_c__uint16__Sequence__fini(&ros_message->monitoring_case_number_outputs);
    }
    if (!rosidl_runtime_c__uint16__Sequence__init(&ros_message->monitoring_case_number_outputs, size)) {
      fprintf(stderr, "failed to create array for field 'monitoring_case_number_outputs'");
      return false;
    }
    auto array_ptr = ros_message->monitoring_case_number_outputs.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: monitoring_case_number_outputs_flags
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

    if (ros_message->monitoring_case_number_outputs_flags.data) {
      rosidl_runtime_c__boolean__Sequence__fini(&ros_message->monitoring_case_number_outputs_flags);
    }
    if (!rosidl_runtime_c__boolean__Sequence__init(&ros_message->monitoring_case_number_outputs_flags, size)) {
      fprintf(stderr, "failed to create array for field 'monitoring_case_number_outputs_flags'");
      return false;
    }
    auto array_ptr = ros_message->monitoring_case_number_outputs_flags.data;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  // Field name: sleep_mode_output
  {
    cdr >> ros_message->sleep_mode_output;
  }

  // Field name: sleep_mode_output_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->sleep_mode_output_valid = tmp ? true : false;
  }

  // Field name: error_flag_contamination_warning
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->error_flag_contamination_warning = tmp ? true : false;
  }

  // Field name: error_flag_contamination_error
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->error_flag_contamination_error = tmp ? true : false;
  }

  // Field name: error_flag_manipulation_error
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->error_flag_manipulation_error = tmp ? true : false;
  }

  // Field name: error_flag_glare
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->error_flag_glare = tmp ? true : false;
  }

  // Field name: error_flag_reference_contour_intruded
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->error_flag_reference_contour_intruded = tmp ? true : false;
  }

  // Field name: error_flag_critical_error
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->error_flag_critical_error = tmp ? true : false;
  }

  // Field name: error_flags_are_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->error_flags_are_valid = tmp ? true : false;
  }

  // Field name: linear_velocity_outputs_velocity_0
  {
    cdr >> ros_message->linear_velocity_outputs_velocity_0;
  }

  // Field name: linear_velocity_outputs_velocity_0_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->linear_velocity_outputs_velocity_0_valid = tmp ? true : false;
  }

  // Field name: linear_velocity_outputs_velocity_0_transmitted_safely
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->linear_velocity_outputs_velocity_0_transmitted_safely = tmp ? true : false;
  }

  // Field name: linear_velocity_outputs_velocity_1
  {
    cdr >> ros_message->linear_velocity_outputs_velocity_1;
  }

  // Field name: linear_velocity_outputs_velocity_1_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->linear_velocity_outputs_velocity_1_valid = tmp ? true : false;
  }

  // Field name: linear_velocity_outputs_velocity_1_transmitted_safely
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->linear_velocity_outputs_velocity_1_transmitted_safely = tmp ? true : false;
  }

  // Field name: resulting_velocity
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

    if (ros_message->resulting_velocity.data) {
      rosidl_runtime_c__int16__Sequence__fini(&ros_message->resulting_velocity);
    }
    if (!rosidl_runtime_c__int16__Sequence__init(&ros_message->resulting_velocity, size)) {
      fprintf(stderr, "failed to create array for field 'resulting_velocity'");
      return false;
    }
    auto array_ptr = ros_message->resulting_velocity.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: resulting_velocity_flags
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

    if (ros_message->resulting_velocity_flags.data) {
      rosidl_runtime_c__boolean__Sequence__fini(&ros_message->resulting_velocity_flags);
    }
    if (!rosidl_runtime_c__boolean__Sequence__init(&ros_message->resulting_velocity_flags, size)) {
      fprintf(stderr, "failed to create array for field 'resulting_velocity_flags'");
      return false;
    }
    auto array_ptr = ros_message->resulting_velocity_flags.data;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sick_safetyscanners2_interfaces
size_t get_serialized_size_sick_safetyscanners2_interfaces__msg__ApplicationOutputs(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ApplicationOutputs__ros_msg_type * ros_message = static_cast<const _ApplicationOutputs__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name evaluation_path_outputs_eval_out
  {
    size_t array_size = ros_message->evaluation_path_outputs_eval_out.size;
    auto array_ptr = ros_message->evaluation_path_outputs_eval_out.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name evaluation_path_outputs_is_safe
  {
    size_t array_size = ros_message->evaluation_path_outputs_is_safe.size;
    auto array_ptr = ros_message->evaluation_path_outputs_is_safe.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name evaluation_path_outputs_is_valid
  {
    size_t array_size = ros_message->evaluation_path_outputs_is_valid.size;
    auto array_ptr = ros_message->evaluation_path_outputs_is_valid.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name monitoring_case_number_outputs
  {
    size_t array_size = ros_message->monitoring_case_number_outputs.size;
    auto array_ptr = ros_message->monitoring_case_number_outputs.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name monitoring_case_number_outputs_flags
  {
    size_t array_size = ros_message->monitoring_case_number_outputs_flags.size;
    auto array_ptr = ros_message->monitoring_case_number_outputs_flags.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name sleep_mode_output
  {
    size_t item_size = sizeof(ros_message->sleep_mode_output);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name sleep_mode_output_valid
  {
    size_t item_size = sizeof(ros_message->sleep_mode_output_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name error_flag_contamination_warning
  {
    size_t item_size = sizeof(ros_message->error_flag_contamination_warning);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name error_flag_contamination_error
  {
    size_t item_size = sizeof(ros_message->error_flag_contamination_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name error_flag_manipulation_error
  {
    size_t item_size = sizeof(ros_message->error_flag_manipulation_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name error_flag_glare
  {
    size_t item_size = sizeof(ros_message->error_flag_glare);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name error_flag_reference_contour_intruded
  {
    size_t item_size = sizeof(ros_message->error_flag_reference_contour_intruded);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name error_flag_critical_error
  {
    size_t item_size = sizeof(ros_message->error_flag_critical_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name error_flags_are_valid
  {
    size_t item_size = sizeof(ros_message->error_flags_are_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name linear_velocity_outputs_velocity_0
  {
    size_t item_size = sizeof(ros_message->linear_velocity_outputs_velocity_0);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name linear_velocity_outputs_velocity_0_valid
  {
    size_t item_size = sizeof(ros_message->linear_velocity_outputs_velocity_0_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name linear_velocity_outputs_velocity_0_transmitted_safely
  {
    size_t item_size = sizeof(ros_message->linear_velocity_outputs_velocity_0_transmitted_safely);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name linear_velocity_outputs_velocity_1
  {
    size_t item_size = sizeof(ros_message->linear_velocity_outputs_velocity_1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name linear_velocity_outputs_velocity_1_valid
  {
    size_t item_size = sizeof(ros_message->linear_velocity_outputs_velocity_1_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name linear_velocity_outputs_velocity_1_transmitted_safely
  {
    size_t item_size = sizeof(ros_message->linear_velocity_outputs_velocity_1_transmitted_safely);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name resulting_velocity
  {
    size_t array_size = ros_message->resulting_velocity.size;
    auto array_ptr = ros_message->resulting_velocity.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name resulting_velocity_flags
  {
    size_t array_size = ros_message->resulting_velocity_flags.size;
    auto array_ptr = ros_message->resulting_velocity_flags.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _ApplicationOutputs__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_sick_safetyscanners2_interfaces__msg__ApplicationOutputs(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sick_safetyscanners2_interfaces
size_t max_serialized_size_sick_safetyscanners2_interfaces__msg__ApplicationOutputs(
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

  // member: evaluation_path_outputs_eval_out
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: evaluation_path_outputs_is_safe
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: evaluation_path_outputs_is_valid
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: monitoring_case_number_outputs
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
  // member: monitoring_case_number_outputs_flags
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: sleep_mode_output
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: sleep_mode_output_valid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: error_flag_contamination_warning
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: error_flag_contamination_error
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: error_flag_manipulation_error
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: error_flag_glare
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: error_flag_reference_contour_intruded
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: error_flag_critical_error
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: error_flags_are_valid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: linear_velocity_outputs_velocity_0
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: linear_velocity_outputs_velocity_0_valid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: linear_velocity_outputs_velocity_0_transmitted_safely
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: linear_velocity_outputs_velocity_1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: linear_velocity_outputs_velocity_1_valid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: linear_velocity_outputs_velocity_1_transmitted_safely
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: resulting_velocity
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
  // member: resulting_velocity_flags
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
    using DataType = sick_safetyscanners2_interfaces__msg__ApplicationOutputs;
    is_plain =
      (
      offsetof(DataType, resulting_velocity_flags) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _ApplicationOutputs__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_sick_safetyscanners2_interfaces__msg__ApplicationOutputs(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ApplicationOutputs = {
  "sick_safetyscanners2_interfaces::msg",
  "ApplicationOutputs",
  _ApplicationOutputs__cdr_serialize,
  _ApplicationOutputs__cdr_deserialize,
  _ApplicationOutputs__get_serialized_size,
  _ApplicationOutputs__max_serialized_size
};

static rosidl_message_type_support_t _ApplicationOutputs__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ApplicationOutputs,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sick_safetyscanners2_interfaces, msg, ApplicationOutputs)() {
  return &_ApplicationOutputs__type_support;
}

#if defined(__cplusplus)
}
#endif
