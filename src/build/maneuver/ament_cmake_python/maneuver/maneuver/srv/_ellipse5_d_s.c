// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from maneuver:srv/Ellipse5D.idl
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
#include "maneuver/srv/detail/ellipse5_d__struct.h"
#include "maneuver/srv/detail/ellipse5_d__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool maneuver__srv__ellipse5_d__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[43];
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
    assert(strncmp("maneuver.srv._ellipse5_d.Ellipse5D_Request", full_classname_dest, 42) == 0);
  }
  maneuver__srv__Ellipse5D_Request * ros_message = _ros_message;
  {  // x_min
    PyObject * field = PyObject_GetAttrString(_pymsg, "x_min");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->x_min = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // x_max
    PyObject * field = PyObject_GetAttrString(_pymsg, "x_max");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->x_max = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // y_min
    PyObject * field = PyObject_GetAttrString(_pymsg, "y_min");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->y_min = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // y_max
    PyObject * field = PyObject_GetAttrString(_pymsg, "y_max");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->y_max = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // z_min
    PyObject * field = PyObject_GetAttrString(_pymsg, "z_min");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->z_min = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // z_max
    PyObject * field = PyObject_GetAttrString(_pymsg, "z_max");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->z_max = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // roll_start
    PyObject * field = PyObject_GetAttrString(_pymsg, "roll_start");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->roll_start = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // roll_mid
    PyObject * field = PyObject_GetAttrString(_pymsg, "roll_mid");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->roll_mid = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // roll_end
    PyObject * field = PyObject_GetAttrString(_pymsg, "roll_end");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->roll_end = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pitch_start
    PyObject * field = PyObject_GetAttrString(_pymsg, "pitch_start");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pitch_start = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pitch_mid
    PyObject * field = PyObject_GetAttrString(_pymsg, "pitch_mid");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pitch_mid = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pitch_end
    PyObject * field = PyObject_GetAttrString(_pymsg, "pitch_end");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pitch_end = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "yaw");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->yaw = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // duration
    PyObject * field = PyObject_GetAttrString(_pymsg, "duration");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->duration = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * maneuver__srv__ellipse5_d__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Ellipse5D_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("maneuver.srv._ellipse5_d");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Ellipse5D_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  maneuver__srv__Ellipse5D_Request * ros_message = (maneuver__srv__Ellipse5D_Request *)raw_ros_message;
  {  // x_min
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->x_min);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x_min", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // x_max
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->x_max);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x_max", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y_min
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->y_min);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y_min", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y_max
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->y_max);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y_max", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // z_min
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->z_min);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z_min", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // z_max
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->z_max);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z_max", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // roll_start
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->roll_start);
    {
      int rc = PyObject_SetAttrString(_pymessage, "roll_start", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // roll_mid
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->roll_mid);
    {
      int rc = PyObject_SetAttrString(_pymessage, "roll_mid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // roll_end
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->roll_end);
    {
      int rc = PyObject_SetAttrString(_pymessage, "roll_end", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pitch_start
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pitch_start);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pitch_start", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pitch_mid
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pitch_mid);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pitch_mid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pitch_end
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pitch_end);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pitch_end", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // yaw
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->yaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "yaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // duration
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->duration);
    {
      int rc = PyObject_SetAttrString(_pymessage, "duration", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

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
// #include "maneuver/srv/detail/ellipse5_d__struct.h"
// already included above
// #include "maneuver/srv/detail/ellipse5_d__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool maneuver__srv__ellipse5_d__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[44];
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
    assert(strncmp("maneuver.srv._ellipse5_d.Ellipse5D_Response", full_classname_dest, 43) == 0);
  }
  maneuver__srv__Ellipse5D_Response * ros_message = _ros_message;
  {  // status
    PyObject * field = PyObject_GetAttrString(_pymsg, "status");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->status = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * maneuver__srv__ellipse5_d__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Ellipse5D_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("maneuver.srv._ellipse5_d");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Ellipse5D_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  maneuver__srv__Ellipse5D_Response * ros_message = (maneuver__srv__Ellipse5D_Response *)raw_ros_message;
  {  // status
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->status ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

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
// #include "maneuver/srv/detail/ellipse5_d__struct.h"
// already included above
// #include "maneuver/srv/detail/ellipse5_d__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes


// end nested array functions include
ROSIDL_GENERATOR_C_IMPORT
bool service_msgs__msg__service_event_info__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * service_msgs__msg__service_event_info__convert_to_py(void * raw_ros_message);
bool maneuver__srv__ellipse5_d__request__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * maneuver__srv__ellipse5_d__request__convert_to_py(void * raw_ros_message);
bool maneuver__srv__ellipse5_d__response__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * maneuver__srv__ellipse5_d__response__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool maneuver__srv__ellipse5_d__event__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[41];
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
    assert(strncmp("maneuver.srv._ellipse5_d.Ellipse5D_Event", full_classname_dest, 40) == 0);
  }
  maneuver__srv__Ellipse5D_Event * ros_message = _ros_message;
  {  // info
    PyObject * field = PyObject_GetAttrString(_pymsg, "info");
    if (!field) {
      return false;
    }
    if (!service_msgs__msg__service_event_info__convert_from_py(field, &ros_message->info)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // request
    PyObject * field = PyObject_GetAttrString(_pymsg, "request");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'request'");
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
    if (!maneuver__srv__Ellipse5D_Request__Sequence__init(&(ros_message->request), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create maneuver__srv__Ellipse5D_Request__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    maneuver__srv__Ellipse5D_Request * dest = ros_message->request.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!maneuver__srv__ellipse5_d__request__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // response
    PyObject * field = PyObject_GetAttrString(_pymsg, "response");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'response'");
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
    if (!maneuver__srv__Ellipse5D_Response__Sequence__init(&(ros_message->response), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create maneuver__srv__Ellipse5D_Response__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    maneuver__srv__Ellipse5D_Response * dest = ros_message->response.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!maneuver__srv__ellipse5_d__response__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * maneuver__srv__ellipse5_d__event__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Ellipse5D_Event */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("maneuver.srv._ellipse5_d");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Ellipse5D_Event");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  maneuver__srv__Ellipse5D_Event * ros_message = (maneuver__srv__Ellipse5D_Event *)raw_ros_message;
  {  // info
    PyObject * field = NULL;
    field = service_msgs__msg__service_event_info__convert_to_py(&ros_message->info);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "info", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // request
    PyObject * field = NULL;
    size_t size = ros_message->request.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    maneuver__srv__Ellipse5D_Request * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->request.data[i]);
      PyObject * pyitem = maneuver__srv__ellipse5_d__request__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "request", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // response
    PyObject * field = NULL;
    size_t size = ros_message->response.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    maneuver__srv__Ellipse5D_Response * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->response.data[i]);
      PyObject * pyitem = maneuver__srv__ellipse5_d__response__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "response", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
