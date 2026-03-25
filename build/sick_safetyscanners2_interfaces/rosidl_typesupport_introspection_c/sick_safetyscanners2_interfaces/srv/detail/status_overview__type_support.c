// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from sick_safetyscanners2_interfaces:srv/StatusOverview.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "sick_safetyscanners2_interfaces/srv/detail/status_overview__rosidl_typesupport_introspection_c.h"
#include "sick_safetyscanners2_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "sick_safetyscanners2_interfaces/srv/detail/status_overview__functions.h"
#include "sick_safetyscanners2_interfaces/srv/detail/status_overview__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void sick_safetyscanners2_interfaces__srv__StatusOverview_Request__rosidl_typesupport_introspection_c__StatusOverview_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  sick_safetyscanners2_interfaces__srv__StatusOverview_Request__init(message_memory);
}

void sick_safetyscanners2_interfaces__srv__StatusOverview_Request__rosidl_typesupport_introspection_c__StatusOverview_Request_fini_function(void * message_memory)
{
  sick_safetyscanners2_interfaces__srv__StatusOverview_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember sick_safetyscanners2_interfaces__srv__StatusOverview_Request__rosidl_typesupport_introspection_c__StatusOverview_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__srv__StatusOverview_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers sick_safetyscanners2_interfaces__srv__StatusOverview_Request__rosidl_typesupport_introspection_c__StatusOverview_Request_message_members = {
  "sick_safetyscanners2_interfaces__srv",  // message namespace
  "StatusOverview_Request",  // message name
  1,  // number of fields
  sizeof(sick_safetyscanners2_interfaces__srv__StatusOverview_Request),
  sick_safetyscanners2_interfaces__srv__StatusOverview_Request__rosidl_typesupport_introspection_c__StatusOverview_Request_message_member_array,  // message members
  sick_safetyscanners2_interfaces__srv__StatusOverview_Request__rosidl_typesupport_introspection_c__StatusOverview_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  sick_safetyscanners2_interfaces__srv__StatusOverview_Request__rosidl_typesupport_introspection_c__StatusOverview_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t sick_safetyscanners2_interfaces__srv__StatusOverview_Request__rosidl_typesupport_introspection_c__StatusOverview_Request_message_type_support_handle = {
  0,
  &sick_safetyscanners2_interfaces__srv__StatusOverview_Request__rosidl_typesupport_introspection_c__StatusOverview_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_sick_safetyscanners2_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, srv, StatusOverview_Request)() {
  if (!sick_safetyscanners2_interfaces__srv__StatusOverview_Request__rosidl_typesupport_introspection_c__StatusOverview_Request_message_type_support_handle.typesupport_identifier) {
    sick_safetyscanners2_interfaces__srv__StatusOverview_Request__rosidl_typesupport_introspection_c__StatusOverview_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &sick_safetyscanners2_interfaces__srv__StatusOverview_Request__rosidl_typesupport_introspection_c__StatusOverview_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "sick_safetyscanners2_interfaces/srv/detail/status_overview__rosidl_typesupport_introspection_c.h"
// already included above
// #include "sick_safetyscanners2_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "sick_safetyscanners2_interfaces/srv/detail/status_overview__functions.h"
// already included above
// #include "sick_safetyscanners2_interfaces/srv/detail/status_overview__struct.h"


// Include directives for member types
// Member `version_c_version`
// Member `current_time`
// Member `error_info_time`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void sick_safetyscanners2_interfaces__srv__StatusOverview_Response__rosidl_typesupport_introspection_c__StatusOverview_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  sick_safetyscanners2_interfaces__srv__StatusOverview_Response__init(message_memory);
}

void sick_safetyscanners2_interfaces__srv__StatusOverview_Response__rosidl_typesupport_introspection_c__StatusOverview_Response_fini_function(void * message_memory)
{
  sick_safetyscanners2_interfaces__srv__StatusOverview_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember sick_safetyscanners2_interfaces__srv__StatusOverview_Response__rosidl_typesupport_introspection_c__StatusOverview_Response_message_member_array[15] = {
  {
    "version_c_version",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__srv__StatusOverview_Response, version_c_version),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "version_major_version_number",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__srv__StatusOverview_Response, version_major_version_number),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "version_minor_version_number",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__srv__StatusOverview_Response, version_minor_version_number),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "version_release_number",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__srv__StatusOverview_Response, version_release_number),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "device_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__srv__StatusOverview_Response, device_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "config_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__srv__StatusOverview_Response, config_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "application_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__srv__StatusOverview_Response, application_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current_time_power_on_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__srv__StatusOverview_Response, current_time_power_on_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__srv__StatusOverview_Response, current_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current_time_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__srv__StatusOverview_Response, current_time_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current_time_date",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__srv__StatusOverview_Response, current_time_date),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_info_code",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__srv__StatusOverview_Response, error_info_code),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_info_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__srv__StatusOverview_Response, error_info_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_info_time_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__srv__StatusOverview_Response, error_info_time_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_info_time_date",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sick_safetyscanners2_interfaces__srv__StatusOverview_Response, error_info_time_date),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers sick_safetyscanners2_interfaces__srv__StatusOverview_Response__rosidl_typesupport_introspection_c__StatusOverview_Response_message_members = {
  "sick_safetyscanners2_interfaces__srv",  // message namespace
  "StatusOverview_Response",  // message name
  15,  // number of fields
  sizeof(sick_safetyscanners2_interfaces__srv__StatusOverview_Response),
  sick_safetyscanners2_interfaces__srv__StatusOverview_Response__rosidl_typesupport_introspection_c__StatusOverview_Response_message_member_array,  // message members
  sick_safetyscanners2_interfaces__srv__StatusOverview_Response__rosidl_typesupport_introspection_c__StatusOverview_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  sick_safetyscanners2_interfaces__srv__StatusOverview_Response__rosidl_typesupport_introspection_c__StatusOverview_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t sick_safetyscanners2_interfaces__srv__StatusOverview_Response__rosidl_typesupport_introspection_c__StatusOverview_Response_message_type_support_handle = {
  0,
  &sick_safetyscanners2_interfaces__srv__StatusOverview_Response__rosidl_typesupport_introspection_c__StatusOverview_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_sick_safetyscanners2_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, srv, StatusOverview_Response)() {
  if (!sick_safetyscanners2_interfaces__srv__StatusOverview_Response__rosidl_typesupport_introspection_c__StatusOverview_Response_message_type_support_handle.typesupport_identifier) {
    sick_safetyscanners2_interfaces__srv__StatusOverview_Response__rosidl_typesupport_introspection_c__StatusOverview_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &sick_safetyscanners2_interfaces__srv__StatusOverview_Response__rosidl_typesupport_introspection_c__StatusOverview_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "sick_safetyscanners2_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "sick_safetyscanners2_interfaces/srv/detail/status_overview__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers sick_safetyscanners2_interfaces__srv__detail__status_overview__rosidl_typesupport_introspection_c__StatusOverview_service_members = {
  "sick_safetyscanners2_interfaces__srv",  // service namespace
  "StatusOverview",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // sick_safetyscanners2_interfaces__srv__detail__status_overview__rosidl_typesupport_introspection_c__StatusOverview_Request_message_type_support_handle,
  NULL  // response message
  // sick_safetyscanners2_interfaces__srv__detail__status_overview__rosidl_typesupport_introspection_c__StatusOverview_Response_message_type_support_handle
};

static rosidl_service_type_support_t sick_safetyscanners2_interfaces__srv__detail__status_overview__rosidl_typesupport_introspection_c__StatusOverview_service_type_support_handle = {
  0,
  &sick_safetyscanners2_interfaces__srv__detail__status_overview__rosidl_typesupport_introspection_c__StatusOverview_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, srv, StatusOverview_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, srv, StatusOverview_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_sick_safetyscanners2_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, srv, StatusOverview)() {
  if (!sick_safetyscanners2_interfaces__srv__detail__status_overview__rosidl_typesupport_introspection_c__StatusOverview_service_type_support_handle.typesupport_identifier) {
    sick_safetyscanners2_interfaces__srv__detail__status_overview__rosidl_typesupport_introspection_c__StatusOverview_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)sick_safetyscanners2_interfaces__srv__detail__status_overview__rosidl_typesupport_introspection_c__StatusOverview_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, srv, StatusOverview_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sick_safetyscanners2_interfaces, srv, StatusOverview_Response)()->data;
  }

  return &sick_safetyscanners2_interfaces__srv__detail__status_overview__rosidl_typesupport_introspection_c__StatusOverview_service_type_support_handle;
}
