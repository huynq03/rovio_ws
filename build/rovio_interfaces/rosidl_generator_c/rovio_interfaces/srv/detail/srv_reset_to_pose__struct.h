// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rovio_interfaces:srv/SrvResetToPose.idl
// generated code does not contain a copyright notice

#ifndef ROVIO_INTERFACES__SRV__DETAIL__SRV_RESET_TO_POSE__STRUCT_H_
#define ROVIO_INTERFACES__SRV__DETAIL__SRV_RESET_TO_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 't_wm'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in srv/SrvResetToPose in the package rovio_interfaces.
typedef struct rovio_interfaces__srv__SrvResetToPose_Request
{
  geometry_msgs__msg__Pose t_wm;
} rovio_interfaces__srv__SrvResetToPose_Request;

// Struct for a sequence of rovio_interfaces__srv__SrvResetToPose_Request.
typedef struct rovio_interfaces__srv__SrvResetToPose_Request__Sequence
{
  rovio_interfaces__srv__SrvResetToPose_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rovio_interfaces__srv__SrvResetToPose_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'nothing'
#include "std_msgs/msg/detail/empty__struct.h"

/// Struct defined in srv/SrvResetToPose in the package rovio_interfaces.
typedef struct rovio_interfaces__srv__SrvResetToPose_Response
{
  std_msgs__msg__Empty nothing;
} rovio_interfaces__srv__SrvResetToPose_Response;

// Struct for a sequence of rovio_interfaces__srv__SrvResetToPose_Response.
typedef struct rovio_interfaces__srv__SrvResetToPose_Response__Sequence
{
  rovio_interfaces__srv__SrvResetToPose_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rovio_interfaces__srv__SrvResetToPose_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVIO_INTERFACES__SRV__DETAIL__SRV_RESET_TO_POSE__STRUCT_H_
