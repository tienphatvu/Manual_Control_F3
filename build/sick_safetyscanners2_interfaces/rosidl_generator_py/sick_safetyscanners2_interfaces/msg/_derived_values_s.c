// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from sick_safetyscanners2_interfaces:msg/DerivedValues.idl
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
#include "sick_safetyscanners2_interfaces/msg/detail/derived_values__struct.h"
#include "sick_safetyscanners2_interfaces/msg/detail/derived_values__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool sick_safetyscanners2_interfaces__msg__derived_values__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[66];
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
    assert(strncmp("sick_safetyscanners2_interfaces.msg._derived_values.DerivedValues", full_classname_dest, 65) == 0);
  }
  sick_safetyscanners2_interfaces__msg__DerivedValues * ros_message = _ros_message;
  {  // multiplication_factor
    PyObject * field = PyObject_GetAttrString(_pymsg, "multiplication_factor");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->multiplication_factor = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // number_of_beams
    PyObject * field = PyObject_GetAttrString(_pymsg, "number_of_beams");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->number_of_beams = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // scan_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "scan_time");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->scan_time = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // start_angle
    PyObject * field = PyObject_GetAttrString(_pymsg, "start_angle");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->start_angle = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // angular_beam_resolution
    PyObject * field = PyObject_GetAttrString(_pymsg, "angular_beam_resolution");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->angular_beam_resolution = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // interbeam_period
    PyObject * field = PyObject_GetAttrString(_pymsg, "interbeam_period");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->interbeam_period = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * sick_safetyscanners2_interfaces__msg__derived_values__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of DerivedValues */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("sick_safetyscanners2_interfaces.msg._derived_values");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "DerivedValues");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  sick_safetyscanners2_interfaces__msg__DerivedValues * ros_message = (sick_safetyscanners2_interfaces__msg__DerivedValues *)raw_ros_message;
  {  // multiplication_factor
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->multiplication_factor);
    {
      int rc = PyObject_SetAttrString(_pymessage, "multiplication_factor", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // number_of_beams
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->number_of_beams);
    {
      int rc = PyObject_SetAttrString(_pymessage, "number_of_beams", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // scan_time
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->scan_time);
    {
      int rc = PyObject_SetAttrString(_pymessage, "scan_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // start_angle
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->start_angle);
    {
      int rc = PyObject_SetAttrString(_pymessage, "start_angle", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // angular_beam_resolution
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->angular_beam_resolution);
    {
      int rc = PyObject_SetAttrString(_pymessage, "angular_beam_resolution", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // interbeam_period
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->interbeam_period);
    {
      int rc = PyObject_SetAttrString(_pymessage, "interbeam_period", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
