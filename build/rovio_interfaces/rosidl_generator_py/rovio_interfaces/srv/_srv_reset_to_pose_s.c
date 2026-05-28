// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rovio_interfaces:srv/SrvResetToPose.idl
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
#include "rovio_interfaces/srv/detail/srv_reset_to_pose__struct.h"
#include "rovio_interfaces/srv/detail/srv_reset_to_pose__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__pose__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__pose__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool rovio_interfaces__srv__srv_reset_to_pose__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[63];
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
    assert(strncmp("rovio_interfaces.srv._srv_reset_to_pose.SrvResetToPose_Request", full_classname_dest, 62) == 0);
  }
  rovio_interfaces__srv__SrvResetToPose_Request * ros_message = _ros_message;
  {  // t_wm
    PyObject * field = PyObject_GetAttrString(_pymsg, "t_wm");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__pose__convert_from_py(field, &ros_message->t_wm)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rovio_interfaces__srv__srv_reset_to_pose__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SrvResetToPose_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rovio_interfaces.srv._srv_reset_to_pose");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SrvResetToPose_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rovio_interfaces__srv__SrvResetToPose_Request * ros_message = (rovio_interfaces__srv__SrvResetToPose_Request *)raw_ros_message;
  {  // t_wm
    PyObject * field = NULL;
    field = geometry_msgs__msg__pose__convert_to_py(&ros_message->t_wm);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "t_wm", field);
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
// #include "rovio_interfaces/srv/detail/srv_reset_to_pose__struct.h"
// already included above
// #include "rovio_interfaces/srv/detail/srv_reset_to_pose__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__empty__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__empty__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool rovio_interfaces__srv__srv_reset_to_pose__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[64];
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
    assert(strncmp("rovio_interfaces.srv._srv_reset_to_pose.SrvResetToPose_Response", full_classname_dest, 63) == 0);
  }
  rovio_interfaces__srv__SrvResetToPose_Response * ros_message = _ros_message;
  {  // nothing
    PyObject * field = PyObject_GetAttrString(_pymsg, "nothing");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__empty__convert_from_py(field, &ros_message->nothing)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rovio_interfaces__srv__srv_reset_to_pose__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SrvResetToPose_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rovio_interfaces.srv._srv_reset_to_pose");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SrvResetToPose_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rovio_interfaces__srv__SrvResetToPose_Response * ros_message = (rovio_interfaces__srv__SrvResetToPose_Response *)raw_ros_message;
  {  // nothing
    PyObject * field = NULL;
    field = std_msgs__msg__empty__convert_to_py(&ros_message->nothing);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "nothing", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
