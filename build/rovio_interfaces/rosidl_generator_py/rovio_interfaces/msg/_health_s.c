// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rovio_interfaces:msg/Health.idl
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
#include "rovio_interfaces/msg/detail/health__struct.h"
#include "rovio_interfaces/msg/detail/health__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool rovio_interfaces__msg__health__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[36];
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
    assert(strncmp("rovio_interfaces.msg._health.Health", full_classname_dest, 35) == 0);
  }
  rovio_interfaces__msg__Health * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // valid_feature_ratio
    PyObject * field = PyObject_GetAttrString(_pymsg, "valid_feature_ratio");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->valid_feature_ratio = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // tracked_feature_ratio
    PyObject * field = PyObject_GetAttrString(_pymsg, "tracked_feature_ratio");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->tracked_feature_ratio = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pixel_covariance_ratio
    PyObject * field = PyObject_GetAttrString(_pymsg, "pixel_covariance_ratio");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pixel_covariance_ratio = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // nis_z_score_rmse
    PyObject * field = PyObject_GetAttrString(_pymsg, "nis_z_score_rmse");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->nis_z_score_rmse = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // accel_deviation
    PyObject * field = PyObject_GetAttrString(_pymsg, "accel_deviation");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->accel_deviation = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // speed_deviation
    PyObject * field = PyObject_GetAttrString(_pymsg, "speed_deviation");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->speed_deviation = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // depth_feature_cov_median
    PyObject * field = PyObject_GetAttrString(_pymsg, "depth_feature_cov_median");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->depth_feature_cov_median = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rovio_interfaces__msg__health__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Health */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rovio_interfaces.msg._health");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Health");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rovio_interfaces__msg__Health * ros_message = (rovio_interfaces__msg__Health *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
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
  {  // valid_feature_ratio
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->valid_feature_ratio);
    {
      int rc = PyObject_SetAttrString(_pymessage, "valid_feature_ratio", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tracked_feature_ratio
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->tracked_feature_ratio);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tracked_feature_ratio", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pixel_covariance_ratio
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pixel_covariance_ratio);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pixel_covariance_ratio", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // nis_z_score_rmse
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->nis_z_score_rmse);
    {
      int rc = PyObject_SetAttrString(_pymessage, "nis_z_score_rmse", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // accel_deviation
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->accel_deviation);
    {
      int rc = PyObject_SetAttrString(_pymessage, "accel_deviation", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // speed_deviation
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->speed_deviation);
    {
      int rc = PyObject_SetAttrString(_pymessage, "speed_deviation", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // depth_feature_cov_median
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->depth_feature_cov_median);
    {
      int rc = PyObject_SetAttrString(_pymessage, "depth_feature_cov_median", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
