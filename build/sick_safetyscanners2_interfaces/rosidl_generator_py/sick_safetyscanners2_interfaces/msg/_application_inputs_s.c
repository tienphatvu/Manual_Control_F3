// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from sick_safetyscanners2_interfaces:msg/ApplicationInputs.idl
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
#include "sick_safetyscanners2_interfaces/msg/detail/application_inputs__struct.h"
#include "sick_safetyscanners2_interfaces/msg/detail/application_inputs__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool sick_safetyscanners2_interfaces__msg__application_inputs__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("sick_safetyscanners2_interfaces.msg._application_inputs.ApplicationInputs", full_classname_dest, 73) == 0);
  }
  sick_safetyscanners2_interfaces__msg__ApplicationInputs * ros_message = _ros_message;
  {  // unsafe_inputs_input_sources
    PyObject * field = PyObject_GetAttrString(_pymsg, "unsafe_inputs_input_sources");
    if (!field) {
      return false;
    }
    if (PyObject_CheckBuffer(field)) {
      // Optimization for converting arrays of primitives
      Py_buffer view;
      int rc = PyObject_GetBuffer(field, &view, PyBUF_SIMPLE);
      if (rc < 0) {
        Py_DECREF(field);
        return false;
      }
      Py_ssize_t size = view.len / sizeof(bool);
      if (!rosidl_runtime_c__boolean__Sequence__init(&(ros_message->unsafe_inputs_input_sources), size)) {
        PyErr_SetString(PyExc_RuntimeError, "unable to create boolean__Sequence ros_message");
        PyBuffer_Release(&view);
        Py_DECREF(field);
        return false;
      }
      bool * dest = ros_message->unsafe_inputs_input_sources.data;
      rc = PyBuffer_ToContiguous(dest, &view, view.len, 'C');
      if (rc < 0) {
        PyBuffer_Release(&view);
        Py_DECREF(field);
        return false;
      }
      PyBuffer_Release(&view);
    } else {
      PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'unsafe_inputs_input_sources'");
      if (!seq_field) {
        Py_DECREF(field);
        return false;
      }
      Py_ssize_t size = PySequence_Size(field);
      if (-1 == size) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
      if (!rosidl_runtime_c__boolean__Sequence__init(&(ros_message->unsafe_inputs_input_sources), size)) {
        PyErr_SetString(PyExc_RuntimeError, "unable to create boolean__Sequence ros_message");
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
      bool * dest = ros_message->unsafe_inputs_input_sources.data;
      for (Py_ssize_t i = 0; i < size; ++i) {
        PyObject * item = PySequence_Fast_GET_ITEM(seq_field, i);
        if (!item) {
          Py_DECREF(seq_field);
          Py_DECREF(field);
          return false;
        }
        assert(PyBool_Check(item));
        bool tmp = (item == Py_True);
        memcpy(&dest[i], &tmp, sizeof(bool));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // unsafe_inputs_flags
    PyObject * field = PyObject_GetAttrString(_pymsg, "unsafe_inputs_flags");
    if (!field) {
      return false;
    }
    if (PyObject_CheckBuffer(field)) {
      // Optimization for converting arrays of primitives
      Py_buffer view;
      int rc = PyObject_GetBuffer(field, &view, PyBUF_SIMPLE);
      if (rc < 0) {
        Py_DECREF(field);
        return false;
      }
      Py_ssize_t size = view.len / sizeof(bool);
      if (!rosidl_runtime_c__boolean__Sequence__init(&(ros_message->unsafe_inputs_flags), size)) {
        PyErr_SetString(PyExc_RuntimeError, "unable to create boolean__Sequence ros_message");
        PyBuffer_Release(&view);
        Py_DECREF(field);
        return false;
      }
      bool * dest = ros_message->unsafe_inputs_flags.data;
      rc = PyBuffer_ToContiguous(dest, &view, view.len, 'C');
      if (rc < 0) {
        PyBuffer_Release(&view);
        Py_DECREF(field);
        return false;
      }
      PyBuffer_Release(&view);
    } else {
      PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'unsafe_inputs_flags'");
      if (!seq_field) {
        Py_DECREF(field);
        return false;
      }
      Py_ssize_t size = PySequence_Size(field);
      if (-1 == size) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
      if (!rosidl_runtime_c__boolean__Sequence__init(&(ros_message->unsafe_inputs_flags), size)) {
        PyErr_SetString(PyExc_RuntimeError, "unable to create boolean__Sequence ros_message");
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
      bool * dest = ros_message->unsafe_inputs_flags.data;
      for (Py_ssize_t i = 0; i < size; ++i) {
        PyObject * item = PySequence_Fast_GET_ITEM(seq_field, i);
        if (!item) {
          Py_DECREF(seq_field);
          Py_DECREF(field);
          return false;
        }
        assert(PyBool_Check(item));
        bool tmp = (item == Py_True);
        memcpy(&dest[i], &tmp, sizeof(bool));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // monitoring_case_number_inputs
    PyObject * field = PyObject_GetAttrString(_pymsg, "monitoring_case_number_inputs");
    if (!field) {
      return false;
    }
    if (PyObject_CheckBuffer(field)) {
      // Optimization for converting arrays of primitives
      Py_buffer view;
      int rc = PyObject_GetBuffer(field, &view, PyBUF_SIMPLE);
      if (rc < 0) {
        Py_DECREF(field);
        return false;
      }
      Py_ssize_t size = view.len / sizeof(uint16_t);
      if (!rosidl_runtime_c__uint16__Sequence__init(&(ros_message->monitoring_case_number_inputs), size)) {
        PyErr_SetString(PyExc_RuntimeError, "unable to create uint16__Sequence ros_message");
        PyBuffer_Release(&view);
        Py_DECREF(field);
        return false;
      }
      uint16_t * dest = ros_message->monitoring_case_number_inputs.data;
      rc = PyBuffer_ToContiguous(dest, &view, view.len, 'C');
      if (rc < 0) {
        PyBuffer_Release(&view);
        Py_DECREF(field);
        return false;
      }
      PyBuffer_Release(&view);
    } else {
      PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'monitoring_case_number_inputs'");
      if (!seq_field) {
        Py_DECREF(field);
        return false;
      }
      Py_ssize_t size = PySequence_Size(field);
      if (-1 == size) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
      if (!rosidl_runtime_c__uint16__Sequence__init(&(ros_message->monitoring_case_number_inputs), size)) {
        PyErr_SetString(PyExc_RuntimeError, "unable to create uint16__Sequence ros_message");
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
      uint16_t * dest = ros_message->monitoring_case_number_inputs.data;
      for (Py_ssize_t i = 0; i < size; ++i) {
        PyObject * item = PySequence_Fast_GET_ITEM(seq_field, i);
        if (!item) {
          Py_DECREF(seq_field);
          Py_DECREF(field);
          return false;
        }
        assert(PyLong_Check(item));
        uint16_t tmp = (uint16_t)PyLong_AsUnsignedLong(item);

        memcpy(&dest[i], &tmp, sizeof(uint16_t));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // monitoring_case_number_inputs_flags
    PyObject * field = PyObject_GetAttrString(_pymsg, "monitoring_case_number_inputs_flags");
    if (!field) {
      return false;
    }
    if (PyObject_CheckBuffer(field)) {
      // Optimization for converting arrays of primitives
      Py_buffer view;
      int rc = PyObject_GetBuffer(field, &view, PyBUF_SIMPLE);
      if (rc < 0) {
        Py_DECREF(field);
        return false;
      }
      Py_ssize_t size = view.len / sizeof(bool);
      if (!rosidl_runtime_c__boolean__Sequence__init(&(ros_message->monitoring_case_number_inputs_flags), size)) {
        PyErr_SetString(PyExc_RuntimeError, "unable to create boolean__Sequence ros_message");
        PyBuffer_Release(&view);
        Py_DECREF(field);
        return false;
      }
      bool * dest = ros_message->monitoring_case_number_inputs_flags.data;
      rc = PyBuffer_ToContiguous(dest, &view, view.len, 'C');
      if (rc < 0) {
        PyBuffer_Release(&view);
        Py_DECREF(field);
        return false;
      }
      PyBuffer_Release(&view);
    } else {
      PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'monitoring_case_number_inputs_flags'");
      if (!seq_field) {
        Py_DECREF(field);
        return false;
      }
      Py_ssize_t size = PySequence_Size(field);
      if (-1 == size) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
      if (!rosidl_runtime_c__boolean__Sequence__init(&(ros_message->monitoring_case_number_inputs_flags), size)) {
        PyErr_SetString(PyExc_RuntimeError, "unable to create boolean__Sequence ros_message");
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
      bool * dest = ros_message->monitoring_case_number_inputs_flags.data;
      for (Py_ssize_t i = 0; i < size; ++i) {
        PyObject * item = PySequence_Fast_GET_ITEM(seq_field, i);
        if (!item) {
          Py_DECREF(seq_field);
          Py_DECREF(field);
          return false;
        }
        assert(PyBool_Check(item));
        bool tmp = (item == Py_True);
        memcpy(&dest[i], &tmp, sizeof(bool));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // linear_velocity_inputs_velocity_0
    PyObject * field = PyObject_GetAttrString(_pymsg, "linear_velocity_inputs_velocity_0");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->linear_velocity_inputs_velocity_0 = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // linear_velocity_inputs_velocity_0_valid
    PyObject * field = PyObject_GetAttrString(_pymsg, "linear_velocity_inputs_velocity_0_valid");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->linear_velocity_inputs_velocity_0_valid = (Py_True == field);
    Py_DECREF(field);
  }
  {  // linear_velocity_inputs_velocity_0_transmitted_safely
    PyObject * field = PyObject_GetAttrString(_pymsg, "linear_velocity_inputs_velocity_0_transmitted_safely");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->linear_velocity_inputs_velocity_0_transmitted_safely = (Py_True == field);
    Py_DECREF(field);
  }
  {  // linear_velocity_inputs_velocity_1
    PyObject * field = PyObject_GetAttrString(_pymsg, "linear_velocity_inputs_velocity_1");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->linear_velocity_inputs_velocity_1 = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // linear_velocity_inputs_velocity_1_valid
    PyObject * field = PyObject_GetAttrString(_pymsg, "linear_velocity_inputs_velocity_1_valid");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->linear_velocity_inputs_velocity_1_valid = (Py_True == field);
    Py_DECREF(field);
  }
  {  // linear_velocity_inputs_velocity_1_transmitted_safely
    PyObject * field = PyObject_GetAttrString(_pymsg, "linear_velocity_inputs_velocity_1_transmitted_safely");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->linear_velocity_inputs_velocity_1_transmitted_safely = (Py_True == field);
    Py_DECREF(field);
  }
  {  // sleep_mode_input
    PyObject * field = PyObject_GetAttrString(_pymsg, "sleep_mode_input");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sleep_mode_input = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * sick_safetyscanners2_interfaces__msg__application_inputs__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ApplicationInputs */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("sick_safetyscanners2_interfaces.msg._application_inputs");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ApplicationInputs");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  sick_safetyscanners2_interfaces__msg__ApplicationInputs * ros_message = (sick_safetyscanners2_interfaces__msg__ApplicationInputs *)raw_ros_message;
  {  // unsafe_inputs_input_sources
    PyObject * field = NULL;
    size_t size = ros_message->unsafe_inputs_input_sources.size;
    bool * src = ros_message->unsafe_inputs_input_sources.data;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    for (size_t i = 0; i < size; ++i) {
      int rc = PyList_SetItem(field, i, PyBool_FromLong(src[i] ? 1 : 0));
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "unsafe_inputs_input_sources", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // unsafe_inputs_flags
    PyObject * field = NULL;
    size_t size = ros_message->unsafe_inputs_flags.size;
    bool * src = ros_message->unsafe_inputs_flags.data;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    for (size_t i = 0; i < size; ++i) {
      int rc = PyList_SetItem(field, i, PyBool_FromLong(src[i] ? 1 : 0));
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "unsafe_inputs_flags", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // monitoring_case_number_inputs
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "monitoring_case_number_inputs");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "array.array") == 0);
    // ensure that itemsize matches the sizeof of the ROS message field
    PyObject * itemsize_attr = PyObject_GetAttrString(field, "itemsize");
    assert(itemsize_attr != NULL);
    size_t itemsize = PyLong_AsSize_t(itemsize_attr);
    Py_DECREF(itemsize_attr);
    if (itemsize != sizeof(uint16_t)) {
      PyErr_SetString(PyExc_RuntimeError, "itemsize doesn't match expectation");
      Py_DECREF(field);
      return NULL;
    }
    // clear the array, poor approach to remove potential default values
    Py_ssize_t length = PyObject_Length(field);
    if (-1 == length) {
      Py_DECREF(field);
      return NULL;
    }
    if (length > 0) {
      PyObject * pop = PyObject_GetAttrString(field, "pop");
      assert(pop != NULL);
      for (Py_ssize_t i = 0; i < length; ++i) {
        PyObject * ret = PyObject_CallFunctionObjArgs(pop, NULL);
        if (!ret) {
          Py_DECREF(pop);
          Py_DECREF(field);
          return NULL;
        }
        Py_DECREF(ret);
      }
      Py_DECREF(pop);
    }
    if (ros_message->monitoring_case_number_inputs.size > 0) {
      // populating the array.array using the frombytes method
      PyObject * frombytes = PyObject_GetAttrString(field, "frombytes");
      assert(frombytes != NULL);
      uint16_t * src = &(ros_message->monitoring_case_number_inputs.data[0]);
      PyObject * data = PyBytes_FromStringAndSize((const char *)src, ros_message->monitoring_case_number_inputs.size * sizeof(uint16_t));
      assert(data != NULL);
      PyObject * ret = PyObject_CallFunctionObjArgs(frombytes, data, NULL);
      Py_DECREF(data);
      Py_DECREF(frombytes);
      if (!ret) {
        Py_DECREF(field);
        return NULL;
      }
      Py_DECREF(ret);
    }
    Py_DECREF(field);
  }
  {  // monitoring_case_number_inputs_flags
    PyObject * field = NULL;
    size_t size = ros_message->monitoring_case_number_inputs_flags.size;
    bool * src = ros_message->monitoring_case_number_inputs_flags.data;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    for (size_t i = 0; i < size; ++i) {
      int rc = PyList_SetItem(field, i, PyBool_FromLong(src[i] ? 1 : 0));
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "monitoring_case_number_inputs_flags", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // linear_velocity_inputs_velocity_0
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->linear_velocity_inputs_velocity_0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "linear_velocity_inputs_velocity_0", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // linear_velocity_inputs_velocity_0_valid
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->linear_velocity_inputs_velocity_0_valid ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "linear_velocity_inputs_velocity_0_valid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // linear_velocity_inputs_velocity_0_transmitted_safely
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->linear_velocity_inputs_velocity_0_transmitted_safely ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "linear_velocity_inputs_velocity_0_transmitted_safely", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // linear_velocity_inputs_velocity_1
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->linear_velocity_inputs_velocity_1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "linear_velocity_inputs_velocity_1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // linear_velocity_inputs_velocity_1_valid
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->linear_velocity_inputs_velocity_1_valid ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "linear_velocity_inputs_velocity_1_valid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // linear_velocity_inputs_velocity_1_transmitted_safely
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->linear_velocity_inputs_velocity_1_transmitted_safely ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "linear_velocity_inputs_velocity_1_transmitted_safely", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sleep_mode_input
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->sleep_mode_input);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sleep_mode_input", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
