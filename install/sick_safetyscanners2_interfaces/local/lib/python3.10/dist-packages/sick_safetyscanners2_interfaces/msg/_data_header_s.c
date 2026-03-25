// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from sick_safetyscanners2_interfaces:msg/DataHeader.idl
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
#include "sick_safetyscanners2_interfaces/msg/detail/data_header__struct.h"
#include "sick_safetyscanners2_interfaces/msg/detail/data_header__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool sick_safetyscanners2_interfaces__msg__data_header__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[60];
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
    assert(strncmp("sick_safetyscanners2_interfaces.msg._data_header.DataHeader", full_classname_dest, 59) == 0);
  }
  sick_safetyscanners2_interfaces__msg__DataHeader * ros_message = _ros_message;
  {  // version_version
    PyObject * field = PyObject_GetAttrString(_pymsg, "version_version");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->version_version = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // version_major_version
    PyObject * field = PyObject_GetAttrString(_pymsg, "version_major_version");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->version_major_version = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // version_minor_version
    PyObject * field = PyObject_GetAttrString(_pymsg, "version_minor_version");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->version_minor_version = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // version_release
    PyObject * field = PyObject_GetAttrString(_pymsg, "version_release");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->version_release = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // serial_number_of_device
    PyObject * field = PyObject_GetAttrString(_pymsg, "serial_number_of_device");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->serial_number_of_device = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // serial_number_of_channel_plug
    PyObject * field = PyObject_GetAttrString(_pymsg, "serial_number_of_channel_plug");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->serial_number_of_channel_plug = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // channel_number
    PyObject * field = PyObject_GetAttrString(_pymsg, "channel_number");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->channel_number = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // sequence_number
    PyObject * field = PyObject_GetAttrString(_pymsg, "sequence_number");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sequence_number = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // scan_number
    PyObject * field = PyObject_GetAttrString(_pymsg, "scan_number");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->scan_number = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // timestamp_date
    PyObject * field = PyObject_GetAttrString(_pymsg, "timestamp_date");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->timestamp_date = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // timestamp_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "timestamp_time");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->timestamp_time = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * sick_safetyscanners2_interfaces__msg__data_header__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of DataHeader */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("sick_safetyscanners2_interfaces.msg._data_header");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "DataHeader");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  sick_safetyscanners2_interfaces__msg__DataHeader * ros_message = (sick_safetyscanners2_interfaces__msg__DataHeader *)raw_ros_message;
  {  // version_version
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->version_version);
    {
      int rc = PyObject_SetAttrString(_pymessage, "version_version", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // version_major_version
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->version_major_version);
    {
      int rc = PyObject_SetAttrString(_pymessage, "version_major_version", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // version_minor_version
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->version_minor_version);
    {
      int rc = PyObject_SetAttrString(_pymessage, "version_minor_version", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // version_release
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->version_release);
    {
      int rc = PyObject_SetAttrString(_pymessage, "version_release", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // serial_number_of_device
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->serial_number_of_device);
    {
      int rc = PyObject_SetAttrString(_pymessage, "serial_number_of_device", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // serial_number_of_channel_plug
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->serial_number_of_channel_plug);
    {
      int rc = PyObject_SetAttrString(_pymessage, "serial_number_of_channel_plug", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // channel_number
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->channel_number);
    {
      int rc = PyObject_SetAttrString(_pymessage, "channel_number", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sequence_number
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->sequence_number);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sequence_number", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // scan_number
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->scan_number);
    {
      int rc = PyObject_SetAttrString(_pymessage, "scan_number", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // timestamp_date
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->timestamp_date);
    {
      int rc = PyObject_SetAttrString(_pymessage, "timestamp_date", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // timestamp_time
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->timestamp_time);
    {
      int rc = PyObject_SetAttrString(_pymessage, "timestamp_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
