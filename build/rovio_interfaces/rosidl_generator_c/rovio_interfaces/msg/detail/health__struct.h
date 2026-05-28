// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rovio_interfaces:msg/Health.idl
// generated code does not contain a copyright notice

#ifndef ROVIO_INTERFACES__MSG__DETAIL__HEALTH__STRUCT_H_
#define ROVIO_INTERFACES__MSG__DETAIL__HEALTH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/Health in the package rovio_interfaces.
typedef struct rovio_interfaces__msg__Health
{
  std_msgs__msg__Header header;
  float valid_feature_ratio;
  float tracked_feature_ratio;
  float pixel_covariance_ratio;
  float nis_z_score_rmse;
  float accel_deviation;
  float speed_deviation;
  float depth_feature_cov_median;
} rovio_interfaces__msg__Health;

// Struct for a sequence of rovio_interfaces__msg__Health.
typedef struct rovio_interfaces__msg__Health__Sequence
{
  rovio_interfaces__msg__Health * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rovio_interfaces__msg__Health__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVIO_INTERFACES__MSG__DETAIL__HEALTH__STRUCT_H_
