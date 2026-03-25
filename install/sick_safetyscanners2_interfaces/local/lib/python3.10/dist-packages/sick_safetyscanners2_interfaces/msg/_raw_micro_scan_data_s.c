// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from sick_safetyscanners2_interfaces:msg/RawMicroScanData.idl
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
#include "sick_safetyscanners2_interfaces/msg/detail/raw_micro_scan_data__struct.h"
#include "sick_safetyscanners2_interfaces/msg/detail/raw_micro_scan_data__functions.h"

bool sick_safetyscanners2_interfaces__msg__data_header__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * sick_safetyscanners2_interfaces__msg__data_header__convert_to_py(void * raw_ros_message);
bool sick_safetyscanners2_interfaces__msg__derived_values__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * sick_safetyscanners2_interfaces__msg__derived_values__convert_to_py(void * raw_ros_message);
bool sick_safetyscanners2_interfaces__msg__general_system_state__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * sick_safetyscanners2_interfaces__msg__general_system_state__convert_to_py(void * raw_ros_message);
bool sick_safetyscanners2_interfaces__msg__measurement_data__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * sick_safetyscanners2_interfaces__msg__measurement_data__convert_to_py(void * raw_ros_message);
bool sick_safetyscanners2_interfaces__msg__intrusion_data__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * sick_safetyscanners2_interfaces__msg__intrusion_data__convert_to_py(void * raw_ros_message);
bool sick_safetyscanners2_interfaces__msg__application_data__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * sick_safetyscanners2_interfaces__msg__application_data__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool sick_safetyscanners2_interfaces__msg__raw_micro_scan_data__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[74];
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
    assert(strncmp("sick_safetyscanners2_interfaces.msg._raw_micro_scan_data.RawMicroScanData", full_classname_dest, 73) == 0);
  }
  sick_safetyscanners2_interfaces__msg__RawMicroScanData * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!sick_safetyscanners2_interfaces__msg__data_header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // derived_values
    PyObject * field = PyObject_GetAttrString(_pymsg, "derived_values");
    if (!field) {
      return false;
    }
    if (!sick_safetyscanners2_interfaces__msg__derived_values__convert_from_py(field, &ros_message->derived_values)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // general_system_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "general_system_state");
    if (!field) {
      return false;
    }
    if (!sick_safetyscanners2_interfaces__msg__general_system_state__convert_from_py(field, &ros_message->general_system_state)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // measurement_data
    PyObject * field = PyObject_GetAttrString(_pymsg, "measurement_data");
    if (!field) {
      return false;
    }
    if (!sick_safetyscanners2_interfaces__msg__measurement_data__convert_from_py(field, &ros_message->measurement_data)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // intrusion_data
    PyObject * field = PyObject_GetAttrString(_pymsg, "intrusion_data");
    if (!field) {
      return false;
    }
    if (!sick_safetyscanners2_interfaces__msg__intrusion_data__convert_from_py(field, &ros_message->intrusion_data)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // application_data
    PyObject * field = PyObject_GetAttrString(_pymsg, "application_data");
    if (!field) {
      return false;
    }
    if (!sick_safetyscanners2_interfaces__msg__application_data__convert_from_py(field, &ros_message->application_data)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * sick_safetyscanners2_interfaces__msg__raw_micro_scan_data__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of RawMicroScanData */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("sick_safetyscanners2_interfaces.msg._raw_micro_scan_data");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "RawMicroScanData");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  sick_safetyscanners2_interfaces__msg__RawMicroScanData * ros_message = (sick_safetyscanners2_interfaces__msg__RawMicroScanData *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = sick_safetyscanners2_interfaces__msg__data_header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // derived_values
    PyObject * field = NULL;
    field = sick_safetyscanners2_interfaces__msg__derived_values__convert_to_py(&ros_message->derived_values);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "derived_values", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // general_system_state
    PyObject * field = NULL;
    field = sick_safetyscanners2_interfaces__msg__general_system_state__convert_to_py(&ros_message->general_system_state);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "general_system_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // measurement_data
    PyObject * field = NULL;
    field = sick_safetyscanners2_interfaces__msg__measurement_data__convert_to_py(&ros_message->measurement_data);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "measurement_data", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // intrusion_data
    PyObject * field = NULL;
    field = sick_safetyscanners2_interfaces__msg__intrusion_data__convert_to_py(&ros_message->intrusion_data);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "intrusion_data", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // application_data
    PyObject * field = NULL;
    field = sick_safetyscanners2_interfaces__msg__application_data__convert_to_py(&ros_message->application_data);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "application_data", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
