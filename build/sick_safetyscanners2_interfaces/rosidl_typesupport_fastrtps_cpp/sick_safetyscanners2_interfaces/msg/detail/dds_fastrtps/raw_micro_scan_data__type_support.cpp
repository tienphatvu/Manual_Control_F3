// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from sick_safetyscanners2_interfaces:msg/RawMicroScanData.idl
// generated code does not contain a copyright notice
#include "sick_safetyscanners2_interfaces/msg/detail/raw_micro_scan_data__rosidl_typesupport_fastrtps_cpp.hpp"
#include "sick_safetyscanners2_interfaces/msg/detail/raw_micro_scan_data__struct.hpp"

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
bool cdr_serialize(
  const sick_safetyscanners2_interfaces::msg::DataHeader &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  sick_safetyscanners2_interfaces::msg::DataHeader &);
size_t get_serialized_size(
  const sick_safetyscanners2_interfaces::msg::DataHeader &,
  size_t current_alignment);
size_t
max_serialized_size_DataHeader(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace sick_safetyscanners2_interfaces

namespace sick_safetyscanners2_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const sick_safetyscanners2_interfaces::msg::DerivedValues &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  sick_safetyscanners2_interfaces::msg::DerivedValues &);
size_t get_serialized_size(
  const sick_safetyscanners2_interfaces::msg::DerivedValues &,
  size_t current_alignment);
size_t
max_serialized_size_DerivedValues(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace sick_safetyscanners2_interfaces

namespace sick_safetyscanners2_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const sick_safetyscanners2_interfaces::msg::GeneralSystemState &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  sick_safetyscanners2_interfaces::msg::GeneralSystemState &);
size_t get_serialized_size(
  const sick_safetyscanners2_interfaces::msg::GeneralSystemState &,
  size_t current_alignment);
size_t
max_serialized_size_GeneralSystemState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace sick_safetyscanners2_interfaces

namespace sick_safetyscanners2_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const sick_safetyscanners2_interfaces::msg::MeasurementData &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  sick_safetyscanners2_interfaces::msg::MeasurementData &);
size_t get_serialized_size(
  const sick_safetyscanners2_interfaces::msg::MeasurementData &,
  size_t current_alignment);
size_t
max_serialized_size_MeasurementData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace sick_safetyscanners2_interfaces

namespace sick_safetyscanners2_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const sick_safetyscanners2_interfaces::msg::IntrusionData &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  sick_safetyscanners2_interfaces::msg::IntrusionData &);
size_t get_serialized_size(
  const sick_safetyscanners2_interfaces::msg::IntrusionData &,
  size_t current_alignment);
size_t
max_serialized_size_IntrusionData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace sick_safetyscanners2_interfaces

namespace sick_safetyscanners2_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const sick_safetyscanners2_interfaces::msg::ApplicationData &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  sick_safetyscanners2_interfaces::msg::ApplicationData &);
size_t get_serialized_size(
  const sick_safetyscanners2_interfaces::msg::ApplicationData &,
  size_t current_alignment);
size_t
max_serialized_size_ApplicationData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace sick_safetyscanners2_interfaces


namespace sick_safetyscanners2_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sick_safetyscanners2_interfaces
cdr_serialize(
  const sick_safetyscanners2_interfaces::msg::RawMicroScanData & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: derived_values
  sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.derived_values,
    cdr);
  // Member: general_system_state
  sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.general_system_state,
    cdr);
  // Member: measurement_data
  sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.measurement_data,
    cdr);
  // Member: intrusion_data
  sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.intrusion_data,
    cdr);
  // Member: application_data
  sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.application_data,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sick_safetyscanners2_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  sick_safetyscanners2_interfaces::msg::RawMicroScanData & ros_message)
{
  // Member: header
  sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: derived_values
  sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.derived_values);

  // Member: general_system_state
  sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.general_system_state);

  // Member: measurement_data
  sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.measurement_data);

  // Member: intrusion_data
  sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.intrusion_data);

  // Member: application_data
  sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.application_data);

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sick_safetyscanners2_interfaces
get_serialized_size(
  const sick_safetyscanners2_interfaces::msg::RawMicroScanData & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: derived_values

  current_alignment +=
    sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.derived_values, current_alignment);
  // Member: general_system_state

  current_alignment +=
    sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.general_system_state, current_alignment);
  // Member: measurement_data

  current_alignment +=
    sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.measurement_data, current_alignment);
  // Member: intrusion_data

  current_alignment +=
    sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.intrusion_data, current_alignment);
  // Member: application_data

  current_alignment +=
    sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.application_data, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sick_safetyscanners2_interfaces
max_serialized_size_RawMicroScanData(
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


  // Member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_DataHeader(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: derived_values
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_DerivedValues(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: general_system_state
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_GeneralSystemState(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: measurement_data
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_MeasurementData(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: intrusion_data
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_IntrusionData(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: application_data
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_ApplicationData(
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
    using DataType = sick_safetyscanners2_interfaces::msg::RawMicroScanData;
    is_plain =
      (
      offsetof(DataType, application_data) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _RawMicroScanData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const sick_safetyscanners2_interfaces::msg::RawMicroScanData *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _RawMicroScanData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<sick_safetyscanners2_interfaces::msg::RawMicroScanData *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _RawMicroScanData__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const sick_safetyscanners2_interfaces::msg::RawMicroScanData *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _RawMicroScanData__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_RawMicroScanData(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _RawMicroScanData__callbacks = {
  "sick_safetyscanners2_interfaces::msg",
  "RawMicroScanData",
  _RawMicroScanData__cdr_serialize,
  _RawMicroScanData__cdr_deserialize,
  _RawMicroScanData__get_serialized_size,
  _RawMicroScanData__max_serialized_size
};

static rosidl_message_type_support_t _RawMicroScanData__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_RawMicroScanData__callbacks,
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
get_message_type_support_handle<sick_safetyscanners2_interfaces::msg::RawMicroScanData>()
{
  return &sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::_RawMicroScanData__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, sick_safetyscanners2_interfaces, msg, RawMicroScanData)() {
  return &sick_safetyscanners2_interfaces::msg::typesupport_fastrtps_cpp::_RawMicroScanData__handle;
}

#ifdef __cplusplus
}
#endif
