// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from sick_safetyscanners2_interfaces:srv/FieldData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "sick_safetyscanners2_interfaces/srv/detail/field_data__rosidl_typesupport_introspection_c.h"
#include "sick_safetyscanners2_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "sick_safetyscanners2_interfaces/srv/detail/field_data__functions.h"
#include "sick_safetyscanners2_interfaces/srv/detail/field_data__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void sick_safetyscanners2_interfaces__srv__FieldData_Request__rosidl_typesupport_introspection_c__FieldData_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  sick_safetyscanners2_interfaces__srv__FieldData_Request__init(message_memory);
}

void sick_safetyscanners2_interfaces__srv__FieldData_Request__rosidl_typesupport_introspection_c__FieldData_Request_fini_function(void * message_memory)
{
  sick_safetyscanners2_interfaces__srv__FieldData_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember sick_safetyscanners2_interfaces__srv__FieldData_Request__rosidl_typesupport_introspection_c__FieldData_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__srv__FieldData_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers sick_safetyscanners2_interfaces__srv__FieldData_Request__rosidl_typesupport_introspection_c__FieldData_Request_message_members = {
  "sick_safetyscanners2_interfaces__srv",  // message namespace
  "FieldData_Request",  // message name
  1,  // number of fields
  sizeof(sick_safetyscanners2_interfaces__srv__FieldData_Request),
  sick_safetyscanners2_interfaces__srv__FieldData_Request__rosidl_typesupport_introspection_c__FieldData_Request_message_member_array,  // message members
  sick_safetyscanners2_interfaces__srv__FieldData_Request__rosidl_typesupport_introspection_c__FieldData_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  sick_safetyscanners2_interfaces__srv__FieldData_Request__rosidl_typesupport_introspection_c__FieldData_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t sick_safetyscanners2_interfaces__srv__FieldData_Request__rosidl_typesupport_introspection_c__FieldData_Request_message_type_support_handle = {
  0,
  &sick_safetyscanners2_interfaces__srv__FieldData_Request__rosidl_typesupport_introspection_c__FieldData_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_sick_safetyscanners2_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, srv, FieldData_Request)() {
  if (!sick_safetyscanners2_interfaces__srv__FieldData_Request__rosidl_typesupport_introspection_c__FieldData_Request_message_type_support_handle.typesupport_identifier) {
    sick_safetyscanners2_interfaces__srv__FieldData_Request__rosidl_typesupport_introspection_c__FieldData_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &sick_safetyscanners2_interfaces__srv__FieldData_Request__rosidl_typesupport_introspection_c__FieldData_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "sick_safetyscanners2_interfaces/srv/detail/field_data__rosidl_typesupport_introspection_c.h"
// already included above
// #include "sick_safetyscanners2_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "sick_safetyscanners2_interfaces/srv/detail/field_data__functions.h"
// already included above
// #include "sick_safetyscanners2_interfaces/srv/detail/field_data__struct.h"


// Include directives for member types
// Member `fields`
#include "sick_safetyscanners2_interfaces/msg/field.h"
// Member `fields`
#include "sick_safetyscanners2_interfaces/msg/detail/field__rosidl_typesupport_introspection_c.h"
// Member `device_name`
#include "rosidl_runtime_c/string_functions.h"
// Member `monitoring_cases`
#include "sick_safetyscanners2_interfaces/msg/monitoring_case.h"
// Member `monitoring_cases`
#include "sick_safetyscanners2_interfaces/msg/detail/monitoring_case__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__FieldData_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  sick_safetyscanners2_interfaces__srv__FieldData_Response__init(message_memory);
}

void sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__FieldData_Response_fini_function(void * message_memory)
{
  sick_safetyscanners2_interfaces__srv__FieldData_Response__fini(message_memory);
}

size_t sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__size_function__FieldData_Response__fields(
  const void * untyped_member)
{
  const sick_safetyscanners2_interfaces__msg__Field__Sequence * member =
    (const sick_safetyscanners2_interfaces__msg__Field__Sequence *)(untyped_member);
  return member->size;
}

const void * sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__get_const_function__FieldData_Response__fields(
  const void * untyped_member, size_t index)
{
  const sick_safetyscanners2_interfaces__msg__Field__Sequence * member =
    (const sick_safetyscanners2_interfaces__msg__Field__Sequence *)(untyped_member);
  return &member->data[index];
}

void * sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__get_function__FieldData_Response__fields(
  void * untyped_member, size_t index)
{
  sick_safetyscanners2_interfaces__msg__Field__Sequence * member =
    (sick_safetyscanners2_interfaces__msg__Field__Sequence *)(untyped_member);
  return &member->data[index];
}

void sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__fetch_function__FieldData_Response__fields(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const sick_safetyscanners2_interfaces__msg__Field * item =
    ((const sick_safetyscanners2_interfaces__msg__Field *)
    sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__get_const_function__FieldData_Response__fields(untyped_member, index));
  sick_safetyscanners2_interfaces__msg__Field * value =
    (sick_safetyscanners2_interfaces__msg__Field *)(untyped_value);
  *value = *item;
}

void sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__assign_function__FieldData_Response__fields(
  void * untyped_member, size_t index, const void * untyped_value)
{
  sick_safetyscanners2_interfaces__msg__Field * item =
    ((sick_safetyscanners2_interfaces__msg__Field *)
    sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__get_function__FieldData_Response__fields(untyped_member, index));
  const sick_safetyscanners2_interfaces__msg__Field * value =
    (const sick_safetyscanners2_interfaces__msg__Field *)(untyped_value);
  *item = *value;
}

bool sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__resize_function__FieldData_Response__fields(
  void * untyped_member, size_t size)
{
  sick_safetyscanners2_interfaces__msg__Field__Sequence * member =
    (sick_safetyscanners2_interfaces__msg__Field__Sequence *)(untyped_member);
  sick_safetyscanners2_interfaces__msg__Field__Sequence__fini(member);
  return sick_safetyscanners2_interfaces__msg__Field__Sequence__init(member, size);
}

size_t sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__size_function__FieldData_Response__monitoring_cases(
  const void * untyped_member)
{
  const sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence * member =
    (const sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence *)(untyped_member);
  return member->size;
}

const void * sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__get_const_function__FieldData_Response__monitoring_cases(
  const void * untyped_member, size_t index)
{
  const sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence * member =
    (const sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence *)(untyped_member);
  return &member->data[index];
}

void * sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__get_function__FieldData_Response__monitoring_cases(
  void * untyped_member, size_t index)
{
  sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence * member =
    (sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence *)(untyped_member);
  return &member->data[index];
}

void sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__fetch_function__FieldData_Response__monitoring_cases(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const sick_safetyscanners2_interfaces__msg__MonitoringCase * item =
    ((const sick_safetyscanners2_interfaces__msg__MonitoringCase *)
    sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__get_const_function__FieldData_Response__monitoring_cases(untyped_member, index));
  sick_safetyscanners2_interfaces__msg__MonitoringCase * value =
    (sick_safetyscanners2_interfaces__msg__MonitoringCase *)(untyped_value);
  *value = *item;
}

void sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__assign_function__FieldData_Response__monitoring_cases(
  void * untyped_member, size_t index, const void * untyped_value)
{
  sick_safetyscanners2_interfaces__msg__MonitoringCase * item =
    ((sick_safetyscanners2_interfaces__msg__MonitoringCase *)
    sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__get_function__FieldData_Response__monitoring_cases(untyped_member, index));
  const sick_safetyscanners2_interfaces__msg__MonitoringCase * value =
    (const sick_safetyscanners2_interfaces__msg__MonitoringCase *)(untyped_value);
  *item = *value;
}

bool sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__resize_function__FieldData_Response__monitoring_cases(
  void * untyped_member, size_t size)
{
  sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence * member =
    (sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence *)(untyped_member);
  sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence__fini(member);
  return sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__FieldData_Response_message_member_array[3] = {
  {
    "fields",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__srv__FieldData_Response, fields),  // bytes offset in struct
    NULL,  // default value
    sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__size_function__FieldData_Response__fields,  // size() function pointer
    sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__get_const_function__FieldData_Response__fields,  // get_const(index) function pointer
    sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__get_function__FieldData_Response__fields,  // get(index) function pointer
    sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__fetch_function__FieldData_Response__fields,  // fetch(index, &value) function pointer
    sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__assign_function__FieldData_Response__fields,  // assign(index, value) function pointer
    sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__resize_function__FieldData_Response__fields  // resize(index) function pointer
  },
  {
    "device_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__srv__FieldData_Response, device_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "monitoring_cases",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__srv__FieldData_Response, monitoring_cases),  // bytes offset in struct
    NULL,  // default value
    sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__size_function__FieldData_Response__monitoring_cases,  // size() function pointer
    sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__get_const_function__FieldData_Response__monitoring_cases,  // get_const(index) function pointer
    sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__get_function__FieldData_Response__monitoring_cases,  // get(index) function pointer
    sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__fetch_function__FieldData_Response__monitoring_cases,  // fetch(index, &value) function pointer
    sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__assign_function__FieldData_Response__monitoring_cases,  // assign(index, value) function pointer
    sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__resize_function__FieldData_Response__monitoring_cases  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__FieldData_Response_message_members = {
  "sick_safetyscanners2_interfaces__srv",  // message namespace
  "FieldData_Response",  // message name
  3,  // number of fields
  sizeof(sick_safetyscanners2_interfaces__srv__FieldData_Response),
  sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__FieldData_Response_message_member_array,  // message members
  sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__FieldData_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__FieldData_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__FieldData_Response_message_type_support_handle = {
  0,
  &sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__FieldData_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_sick_safetyscanners2_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, srv, FieldData_Response)() {
  sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__FieldData_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, msg, Field)();
  sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__FieldData_Response_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, msg, MonitoringCase)();
  if (!sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__FieldData_Response_message_type_support_handle.typesupport_identifier) {
    sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__FieldData_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &sick_safetyscanners2_interfaces__srv__FieldData_Response__rosidl_typesupport_introspection_c__FieldData_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "sick_safetyscanners2_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "sick_safetyscanners2_interfaces/srv/detail/field_data__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers sick_safetyscanners2_interfaces__srv__detail__field_data__rosidl_typesupport_introspection_c__FieldData_service_members = {
  "sick_safetyscanners2_interfaces__srv",  // service namespace
  "FieldData",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // sick_safetyscanners2_interfaces__srv__detail__field_data__rosidl_typesupport_introspection_c__FieldData_Request_message_type_support_handle,
  NULL  // response message
  // sick_safetyscanners2_interfaces__srv__detail__field_data__rosidl_typesupport_introspection_c__FieldData_Response_message_type_support_handle
};

static rosidl_service_type_support_t sick_safetyscanners2_interfaces__srv__detail__field_data__rosidl_typesupport_introspection_c__FieldData_service_type_support_handle = {
  0,
  &sick_safetyscanners2_interfaces__srv__detail__field_data__rosidl_typesupport_introspection_c__FieldData_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, srv, FieldData_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, srv, FieldData_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_sick_safetyscanners2_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, srv, FieldData)() {
  if (!sick_safetyscanners2_interfaces__srv__detail__field_data__rosidl_typesupport_introspection_c__FieldData_service_type_support_handle.typesupport_identifier) {
    sick_safetyscanners2_interfaces__srv__detail__field_data__rosidl_typesupport_introspection_c__FieldData_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)sick_safetyscanners2_interfaces__srv__detail__field_data__rosidl_typesupport_introspection_c__FieldData_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, srv, FieldData_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, srv, FieldData_Response)()->data;
  }

  return &sick_safetyscanners2_interfaces__srv__detail__field_data__rosidl_typesupport_introspection_c__FieldData_service_type_support_handle;
}
