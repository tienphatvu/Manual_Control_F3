// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from sick_safetyscanners2_interfaces:srv/StatusOverview.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "sick_safetyscanners2_interfaces/srv/detail/status_overview__struct.h"
#include "sick_safetyscanners2_interfaces/srv/detail/status_overview__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool sick_safetyscanners2_interfaces__srv__status_overview__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[76];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("sick_safetyscanners2_interfaces.srv._status_overview.StatusOverview_Request", full_classname_dest, 75) == 0);
  }
  sick_safetyscanners2_interfaces__srv__StatusOverview_Request * ros_message = _ros_message;
  ros_message->structure_needs_at_least_one_member = 0;

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * sick_safetyscanners2_interfaces__srv__status_overview__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of StatusOverview_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("sick_safetyscanners2_interfaces.srv._status_overview");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "StatusOverview_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  (void)raw_ros_message;

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "sick_safetyscanners2_interfaces/srv/detail/status_overview__struct.h"
// already included above
// #include "sick_safetyscanners2_interfaces/srv/detail/status_overview__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool sick_safetyscanners2_interfaces__srv__status_overview__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[77];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("sick_safetyscanners2_interfaces.srv._status_overview.StatusOverview_Response", full_classname_dest, 76) == 0);
  }
  sick_safetyscanners2_interfaces__srv__StatusOverview_Response * ros_message = _ros_message;
  {  // version_c_version
    PyObject * field = PyObject_GetAttrString(_pymsg, "version_c_version");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->version_c_version, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // version_major_version_number
    PyObject * field = PyObject_GetAttrString(_pymsg, "version_major_version_number");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->version_major_version_number = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // version_minor_version_number
    PyObject * field = PyObject_GetAttrString(_pymsg, "version_minor_version_number");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->version_minor_version_number = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // version_release_number
    PyObject * field = PyObject_GetAttrString(_pymsg, "version_release_number");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->version_release_number = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // device_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "device_state");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->device_state = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // config_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "config_state");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->config_state = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // application_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "application_state");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->application_state = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // current_time_power_on_count
    PyObject * field = PyObject_GetAttrString(_pymsg, "current_time_power_on_count");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->current_time_power_on_count = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // current_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "current_time");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->current_time, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // current_time_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "current_time_time");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->current_time_time = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // current_time_date
    PyObject * field = PyObject_GetAttrString(_pymsg, "current_time_date");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->current_time_date = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // error_info_code
    PyObject * field = PyObject_GetAttrString(_pymsg, "error_info_code");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->error_info_code = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // error_info_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "error_info_time");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->error_info_time, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // error_info_time_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "error_info_time_time");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->error_info_time_time = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // error_info_time_date
    PyObject * field = PyObject_GetAttrString(_pymsg, "error_info_time_date");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->error_info_time_date = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * sick_safetyscanners2_interfaces__srv__status_overview__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of StatusOverview_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("sick_safetyscanners2_interfaces.srv._status_overview");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "StatusOverview_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  sick_safetyscanners2_interfaces__srv__StatusOverview_Response * ros_message = (sick_safetyscanners2_interfaces__srv__StatusOverview_Response *)raw_ros_message;
  {  // version_c_version
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->version_c_version.data,
      strlen(ros_message->version_c_version.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "version_c_version", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // version_major_version_number
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->version_major_version_number);
    {
      int rc = PyObject_SetAttrString(_pymessage, "version_major_version_number", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // version_minor_version_number
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->version_minor_version_number);
    {
      int rc = PyObject_SetAttrString(_pymessage, "version_minor_version_number", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // version_release_number
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->version_release_number);
    {
      int rc = PyObject_SetAttrString(_pymessage, "version_release_number", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // device_state
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->device_state);
    {
      int rc = PyObject_SetAttrString(_pymessage, "device_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // config_state
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->config_state);
    {
      int rc = PyObject_SetAttrString(_pymessage, "config_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // application_state
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->application_state);
    {
      int rc = PyObject_SetAttrString(_pymessage, "application_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current_time_power_on_count
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->current_time_power_on_count);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current_time_power_on_count", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current_time
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->current_time.data,
      strlen(ros_message->current_time.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "current_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current_time_time
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->current_time_time);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current_time_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current_time_date
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->current_time_date);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current_time_date", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // error_info_code
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->error_info_code);
    {
      int rc = PyObject_SetAttrString(_pymessage, "error_info_code", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // error_info_time
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->error_info_time.data,
      strlen(ros_message->error_info_time.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "error_info_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // error_info_time_time
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->error_info_time_time);
    {
      int rc = PyObject_SetAttrString(_pymessage, "error_info_time_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // error_info_time_date
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->error_info_time_date);
    {
      int rc = PyObject_SetAttrString(_pymessage, "error_info_time_date", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
