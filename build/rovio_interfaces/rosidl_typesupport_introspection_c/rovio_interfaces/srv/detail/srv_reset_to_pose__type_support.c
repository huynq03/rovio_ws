// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rovio_interfaces:srv/SrvResetToPose.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rovio_interfaces/srv/detail/srv_reset_to_pose__rosidl_typesupport_introspection_c.h"
#include "rovio_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rovio_interfaces/srv/detail/srv_reset_to_pose__functions.h"
#include "rovio_interfaces/srv/detail/srv_reset_to_pose__struct.h"


// Include directives for member types
// Member `t_wm`
#include "geometry_msgs/msg/pose.h"
// Member `t_wm`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rovio_interfaces__srv__SrvResetToPose_Request__rosidl_typesupport_introspection_c__SrvResetToPose_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rovio_interfaces__srv__SrvResetToPose_Request__init(message_memory);
}

void rovio_interfaces__srv__SrvResetToPose_Request__rosidl_typesupport_introspection_c__SrvResetToPose_Request_fini_function(void * message_memory)
{
  rovio_interfaces__srv__SrvResetToPose_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rovio_interfaces__srv__SrvResetToPose_Request__rosidl_typesupport_introspection_c__SrvResetToPose_Request_message_member_array[1] = {
  {
    "t_wm",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rovio_interfaces__srv__SrvResetToPose_Request, t_wm),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rovio_interfaces__srv__SrvResetToPose_Request__rosidl_typesupport_introspection_c__SrvResetToPose_Request_message_members = {
  "rovio_interfaces__srv",  // message namespace
  "SrvResetToPose_Request",  // message name
  1,  // number of fields
  sizeof(rovio_interfaces__srv__SrvResetToPose_Request),
  rovio_interfaces__srv__SrvResetToPose_Request__rosidl_typesupport_introspection_c__SrvResetToPose_Request_message_member_array,  // message members
  rovio_interfaces__srv__SrvResetToPose_Request__rosidl_typesupport_introspection_c__SrvResetToPose_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  rovio_interfaces__srv__SrvResetToPose_Request__rosidl_typesupport_introspection_c__SrvResetToPose_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rovio_interfaces__srv__SrvResetToPose_Request__rosidl_typesupport_introspection_c__SrvResetToPose_Request_message_type_support_handle = {
  0,
  &rovio_interfaces__srv__SrvResetToPose_Request__rosidl_typesupport_introspection_c__SrvResetToPose_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rovio_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rovio_interfaces, srv, SrvResetToPose_Request)() {
  rovio_interfaces__srv__SrvResetToPose_Request__rosidl_typesupport_introspection_c__SrvResetToPose_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  if (!rovio_interfaces__srv__SrvResetToPose_Request__rosidl_typesupport_introspection_c__SrvResetToPose_Request_message_type_support_handle.typesupport_identifier) {
    rovio_interfaces__srv__SrvResetToPose_Request__rosidl_typesupport_introspection_c__SrvResetToPose_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rovio_interfaces__srv__SrvResetToPose_Request__rosidl_typesupport_introspection_c__SrvResetToPose_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "rovio_interfaces/srv/detail/srv_reset_to_pose__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rovio_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rovio_interfaces/srv/detail/srv_reset_to_pose__functions.h"
// already included above
// #include "rovio_interfaces/srv/detail/srv_reset_to_pose__struct.h"


// Include directives for member types
// Member `nothing`
#include "std_msgs/msg/empty.h"
// Member `nothing`
#include "std_msgs/msg/detail/empty__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rovio_interfaces__srv__SrvResetToPose_Response__rosidl_typesupport_introspection_c__SrvResetToPose_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rovio_interfaces__srv__SrvResetToPose_Response__init(message_memory);
}

void rovio_interfaces__srv__SrvResetToPose_Response__rosidl_typesupport_introspection_c__SrvResetToPose_Response_fini_function(void * message_memory)
{
  rovio_interfaces__srv__SrvResetToPose_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rovio_interfaces__srv__SrvResetToPose_Response__rosidl_typesupport_introspection_c__SrvResetToPose_Response_message_member_array[1] = {
  {
    "nothing",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rovio_interfaces__srv__SrvResetToPose_Response, nothing),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rovio_interfaces__srv__SrvResetToPose_Response__rosidl_typesupport_introspection_c__SrvResetToPose_Response_message_members = {
  "rovio_interfaces__srv",  // message namespace
  "SrvResetToPose_Response",  // message name
  1,  // number of fields
  sizeof(rovio_interfaces__srv__SrvResetToPose_Response),
  rovio_interfaces__srv__SrvResetToPose_Response__rosidl_typesupport_introspection_c__SrvResetToPose_Response_message_member_array,  // message members
  rovio_interfaces__srv__SrvResetToPose_Response__rosidl_typesupport_introspection_c__SrvResetToPose_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  rovio_interfaces__srv__SrvResetToPose_Response__rosidl_typesupport_introspection_c__SrvResetToPose_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rovio_interfaces__srv__SrvResetToPose_Response__rosidl_typesupport_introspection_c__SrvResetToPose_Response_message_type_support_handle = {
  0,
  &rovio_interfaces__srv__SrvResetToPose_Response__rosidl_typesupport_introspection_c__SrvResetToPose_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rovio_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rovio_interfaces, srv, SrvResetToPose_Response)() {
  rovio_interfaces__srv__SrvResetToPose_Response__rosidl_typesupport_introspection_c__SrvResetToPose_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Empty)();
  if (!rovio_interfaces__srv__SrvResetToPose_Response__rosidl_typesupport_introspection_c__SrvResetToPose_Response_message_type_support_handle.typesupport_identifier) {
    rovio_interfaces__srv__SrvResetToPose_Response__rosidl_typesupport_introspection_c__SrvResetToPose_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rovio_interfaces__srv__SrvResetToPose_Response__rosidl_typesupport_introspection_c__SrvResetToPose_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rovio_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rovio_interfaces/srv/detail/srv_reset_to_pose__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers rovio_interfaces__srv__detail__srv_reset_to_pose__rosidl_typesupport_introspection_c__SrvResetToPose_service_members = {
  "rovio_interfaces__srv",  // service namespace
  "SrvResetToPose",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // rovio_interfaces__srv__detail__srv_reset_to_pose__rosidl_typesupport_introspection_c__SrvResetToPose_Request_message_type_support_handle,
  NULL  // response message
  // rovio_interfaces__srv__detail__srv_reset_to_pose__rosidl_typesupport_introspection_c__SrvResetToPose_Response_message_type_support_handle
};

static rosidl_service_type_support_t rovio_interfaces__srv__detail__srv_reset_to_pose__rosidl_typesupport_introspection_c__SrvResetToPose_service_type_support_handle = {
  0,
  &rovio_interfaces__srv__detail__srv_reset_to_pose__rosidl_typesupport_introspection_c__SrvResetToPose_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rovio_interfaces, srv, SrvResetToPose_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rovio_interfaces, srv, SrvResetToPose_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rovio_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rovio_interfaces, srv, SrvResetToPose)() {
  if (!rovio_interfaces__srv__detail__srv_reset_to_pose__rosidl_typesupport_introspection_c__SrvResetToPose_service_type_support_handle.typesupport_identifier) {
    rovio_interfaces__srv__detail__srv_reset_to_pose__rosidl_typesupport_introspection_c__SrvResetToPose_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)rovio_interfaces__srv__detail__srv_reset_to_pose__rosidl_typesupport_introspection_c__SrvResetToPose_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rovio_interfaces, srv, SrvResetToPose_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rovio_interfaces, srv, SrvResetToPose_Response)()->data;
  }

  return &rovio_interfaces__srv__detail__srv_reset_to_pose__rosidl_typesupport_introspection_c__SrvResetToPose_service_type_support_handle;
}
