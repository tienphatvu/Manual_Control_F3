// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from sick_safetyscanners2_interfaces:msg/GeneralSystemState.idl
// generated code does not contain a copyright notice
#include "sick_safetyscanners2_interfaces/msg/detail/general_system_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include "sick_safetyscanners2_interfaces/msg/detail/general_system_state__struct.hpp"

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
  const sick_safetyscanners2_interfaces::msg::GeneralSystemState & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: run_mode_active
  cdr << (ros_message.run_mode_active ? true : false);
  // Member: standby_mode_active
  cdr << (ros_message.standby_mode_active ? true : false);
  // Member: contamination_warning
  cdr << (ros_message.contamination_warning ? true : false);
  // Member: contamination_error
  cdr << (ros_message.contamination_error ? true : false);
  // Member: reference_contour_status
  cdr << (ros_message.reference_contour_status ? true : false);
  // Member: manipulation_status
  cdr << (ros_message.manipulation_status ? true : false);
  // Member: safe_cut_off_path
  {
    cdr << ros_message.safe_cut_off_path;
  }
  // Member: non_safe_cut_off_path
  {
    cdr << ros_message.non_safe_cut_off_path;
  }
  // Member: reset_required_cut_off_path
  {
    cdr << ros_message.reset_required_cut_off_path;
  }
  // Member: current_monitoring_case_no_table_1
  cdr << ros_message.current_monitoring_case_no_table_1;
  // Member: current_monitoring_case_no_table_2
  cdr << ros_message.current_monitoring_case_no_table_2;
  // Member: current_monitoring_case_no_table_3
  cdr << ros_message.current_monitoring_case_no_table_3;
  // Member: current_monitoring_case_no_table_4
  cdr << ros_message.current_monitoring_case_no_table_4;
  // Member: application_error
  cdr << (ros_message.application_error ? true : false);
  // Member: device_error
  cdr << (ros_message.device_error ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sick_safetyscanners2_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  sick_safetyscanners2_interfaces::msg::GeneralSystemState & ros_message)
{
  // Member: run_mode_active
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.run_mode_active = tmp ? true : false;
  }

  // Member: standby_mode_active
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.standby_mode_active = tmp ? true : false;
  }

  // Member: contamination_warning
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.contamination_warning = tmp ? true : false;
  }

  // Member: contamination_error
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.contamination_error = tmp ? true : false;
  }

  // Member: reference_contour_status
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.reference_contour_status = tmp ? true : false;
  }

  // Member: manipulation_status
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.manipulation_status = tmp ? true : false;
  }

  // Member: safe_cut_off_path
  {
    cdr >> ros_message.safe_cut_off_path;
  }

  // Member: non_safe_cut_off_path
  {
    cdr >> ros_message.non_safe_cut_off_path;
  }

  // Member: reset_required_cut_off_path
  {
    cdr >> ros_message.reset_required_cut_off_path;
  }

  // Member: current_monitoring_case_no_table_1
  cdr >> ros_message.current_monitoring_case_no_table_1;

  // Member: current_monitoring_case_no_table_2
  cdr >> ros_message.current_monitoring_case_no_table_2;

  // Member: current_monitoring_case_no_table_3
  cdr >> ros_message.current_monitoring_case_no_table_3;

  // Member: current_monitoring_case_no_table_4
  cdr >> ros_message.current_monitoring_case_no_table_4;

  // Member: application_error
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.application_error = tmp ? true : false;
  }

  // Member: device_error
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.device_error = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sick_safetyscanners2_interfaces
get_serialized_size(
  const sick_safetyscanners2_interfaces::msg::GeneralSystemState & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: run_mode_active
  {
    size_t item_size = sizeof(ros_message.run_mode_active);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: standby_mode_active
  {
    size_t item_size = sizeof(ros_message.standby_mode_active);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: contamination_warning
  {
    size_t item_size = sizeof(ros_message.contamination_warning);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: contamination_error
  {
    size_t item_size = sizeof(ros_message.contamination_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: reference_contour_status
  {
    size_t item_size = sizeof(ros_message.reference_contour_status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: manipulation_status
  {
    size_t item_size = sizeof(ros_message.manipulation_status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: safe_cut_off_path
  {
    size_t array_size = ros_message.safe_cut_off_path.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.safe_cut_off_path[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: non_safe_cut_off_path
  {
    size_t array_size = ros_message.non_safe_cut_off_path.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.non_safe_cut_off_path[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: reset_required_cut_off_path
  {
    size_t array_size = ros_message.reset_required_cut_off_path.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.reset_required_cut_off_path[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: current_monitoring_case_no_table_1
  {
    size_t item_size = sizeof(ros_message.current_monitoring_case_no_table_1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: current_monitoring_case_no_table_2
  {
    size_t item_size = sizeof(ros_message.current_monitoring_case_no_table_2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: current_monitoring_case_no_table_3
  {
    size_t item_size = sizeof(ros_message.current_monitoring_case_no_table_3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: current_monitoring_case_no_table_4
  {
    size_t item_size = sizeof(ros_message.current_monitoring_case_no_table_4);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: application_error
  {
    size_t item_size = sizeof(ros_message.application_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: device_error
  {
    size_t item_size = sizeof(ros_message.device_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sick_safetyscanners2_interfaces
max_serialized_size_GeneralSystemState(
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


  // Member: run_mode_active
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: standby_mode_active
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

  // Member: contamination_error
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: reference_contour_status
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: manipulation_status
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: safe_cut_off_path
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: non_safe_cut_off_path
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: reset_required_cut_off_path
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: current_monitoring_case_no_table_1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: current_monitoring_case_no_table_2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: current_monitoring_case_no_table_3
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: current_monitoring_case_no_table_4
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: application_error
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: device_error
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
    using DataType = sick_safetyscanners2_interfaces::msg::GeneralSystemState;
    is_plain =
      (
      offsetof(DataType, device_error) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _GeneralSystemState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const sick_safetyscanners2_interfaces::msg::GeneralSystemState *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _GeneralSystemState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<sick_safetyscanners2_interfaces::msg::GeneralSystemState *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _GeneralSystemState__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const sick_safetyscanners2_interfaces::msg::GeneralSystemState *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _GeneralSystemState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_GeneralSystemState(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _GeneralSystemState__callbacks = {
  "sick_safetyscanners2_interfaces::msg",
  "GeneralSystemState",
  _GeneralSystemState__cdr_serialize,
  _GeneralSystemState__cdr_deserialize,
  _GeneralSystemState__get_serialized_size,
  _GeneralSystemState__max_serialized_size
};

static rosidl_message_type_support_t _GeneralSystemState__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_GeneralSystemState__callbacks,
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
get_message_type_support_handle<sick_safetyscanners2_interfaces::msg::GeneralSystemState>()
{
  return &sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::_GeneralSystemState__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, sick_safetyscanners2_interfaces, msg, GeneralSystemState)() {
  return &sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::_GeneralSystemState__handle;
}

#ifdef __cplusplus
}
#endif
