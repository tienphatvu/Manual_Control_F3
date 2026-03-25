// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from sick_safetyscanners2_interfaces:msg/ScanPoint.idl
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
#include "sick_safetyscanners2_interfaces/msg/detail/scan_point__struct.h"
#include "sick_safetyscanners2_interfaces/msg/detail/scan_point__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool sick_safetyscanners2_interfaces__msg__scan_point__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[58];
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
    assert(strncmp("sick_safetyscanners2_interfaces.msg._scan_point.ScanPoint", full_classname_dest, 57) == 0);
  }
  sick_safetyscanners2_interfaces__msg__ScanPoint * ros_message = _ros_message;
  {  // angle
    PyObject * field = PyObject_GetAttrString(_pymsg, "angle");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->angle = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // distance
    PyObject * field = PyObject_GetAttrString(_pymsg, "distance");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->distance = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // reflectivity
    PyObject * field = PyObject_GetAttrString(_pymsg, "reflectivity");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->reflectivity = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // valid
    PyObject * field = PyObject_GetAttrString(_pymsg, "valid");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->valid = (Py_True == field);
    Py_DECREF(field);
  }
  {  // infinite
    PyObject * field = PyObject_GetAttrString(_pymsg, "infinite");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->infinite = (Py_True == field);
    Py_DECREF(field);
  }
  {  // glare
    PyObject * field = PyObject_GetAttrString(_pymsg, "glare");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->glare = (Py_True == field);
    Py_DECREF(field);
  }
  {  // reflector
    PyObject * field = PyObject_GetAttrString(_pymsg, "reflector");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->reflector = (Py_True == field);
    Py_DECREF(field);
  }
  {  // contamination
    PyObject * field = PyObject_GetAttrString(_pymsg, "contamination");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->contamination = (Py_True == field);
    Py_DECREF(field);
  }
  {  // contamination_warning
    PyObject * field = PyObject_GetAttrString(_pymsg, "contamination_warning");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->contamination_warning = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * sick_safetyscanners2_interfaces__msg__scan_point__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ScanPoint */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("sick_safetyscanners2_interfaces.msg._scan_point");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ScanPoint");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  sick_safetyscanners2_interfaces__msg__ScanPoint * ros_message = (sick_safetyscanners2_interfaces__msg__ScanPoint *)raw_ros_message;
  {  // angle
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->angle);
    {
      int rc = PyObject_SetAttrString(_pymessage, "angle", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // distance
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->distance);
    {
      int rc = PyObject_SetAttrString(_pymessage, "distance", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // reflectivity
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->reflectivity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "reflectivity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // valid
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->valid ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "valid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // infinite
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->infinite ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "infinite", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // glare
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->glare ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "glare", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // reflector
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->reflector ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "reflector", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // contamination
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->contamination ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "contamination", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // contamination_warning
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->contamination_warning ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "contamination_warning", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
