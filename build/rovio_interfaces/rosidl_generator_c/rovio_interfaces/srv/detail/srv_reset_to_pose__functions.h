// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rovio_interfaces:srv/SrvResetToPose.idl
// generated code does not contain a copyright notice

#ifndef ROVIO_INTERFACES__SRV__DETAIL__SRV_RESET_TO_POSE__FUNCTIONS_H_
#define ROVIO_INTERFACES__SRV__DETAIL__SRV_RESET_TO_POSE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "rovio_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "rovio_interfaces/srv/detail/srv_reset_to_pose__struct.h"

/// Initialize srv/SrvResetToPose message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rovio_interfaces__srv__SrvResetToPose_Request
 * )) before or use
 * rovio_interfaces__srv__SrvResetToPose_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rovio_interfaces
bool
rovio_interfaces__srv__SrvResetToPose_Request__init(rovio_interfaces__srv__SrvResetToPose_Request * msg);

/// Finalize srv/SrvResetToPose message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rovio_interfaces
void
rovio_interfaces__srv__SrvResetToPose_Request__fini(rovio_interfaces__srv__SrvResetToPose_Request * msg);

/// Create srv/SrvResetToPose message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rovio_interfaces__srv__SrvResetToPose_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rovio_interfaces
rovio_interfaces__srv__SrvResetToPose_Request *
rovio_interfaces__srv__SrvResetToPose_Request__create();

/// Destroy srv/SrvResetToPose message.
/**
 * It calls
 * rovio_interfaces__srv__SrvResetToPose_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rovio_interfaces
void
rovio_interfaces__srv__SrvResetToPose_Request__destroy(rovio_interfaces__srv__SrvResetToPose_Request * msg);

/// Check for srv/SrvResetToPose message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rovio_interfaces
bool
rovio_interfaces__srv__SrvResetToPose_Request__are_equal(const rovio_interfaces__srv__SrvResetToPose_Request * lhs, const rovio_interfaces__srv__SrvResetToPose_Request * rhs);

/// Copy a srv/SrvResetToPose message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rovio_interfaces
bool
rovio_interfaces__srv__SrvResetToPose_Request__copy(
  const rovio_interfaces__srv__SrvResetToPose_Request * input,
  rovio_interfaces__srv__SrvResetToPose_Request * output);

/// Initialize array of srv/SrvResetToPose messages.
/**
 * It allocates the memory for the number of elements and calls
 * rovio_interfaces__srv__SrvResetToPose_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rovio_interfaces
bool
rovio_interfaces__srv__SrvResetToPose_Request__Sequence__init(rovio_interfaces__srv__SrvResetToPose_Request__Sequence * array, size_t size);

/// Finalize array of srv/SrvResetToPose messages.
/**
 * It calls
 * rovio_interfaces__srv__SrvResetToPose_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rovio_interfaces
void
rovio_interfaces__srv__SrvResetToPose_Request__Sequence__fini(rovio_interfaces__srv__SrvResetToPose_Request__Sequence * array);

/// Create array of srv/SrvResetToPose messages.
/**
 * It allocates the memory for the array and calls
 * rovio_interfaces__srv__SrvResetToPose_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rovio_interfaces
rovio_interfaces__srv__SrvResetToPose_Request__Sequence *
rovio_interfaces__srv__SrvResetToPose_Request__Sequence__create(size_t size);

/// Destroy array of srv/SrvResetToPose messages.
/**
 * It calls
 * rovio_interfaces__srv__SrvResetToPose_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rovio_interfaces
void
rovio_interfaces__srv__SrvResetToPose_Request__Sequence__destroy(rovio_interfaces__srv__SrvResetToPose_Request__Sequence * array);

/// Check for srv/SrvResetToPose message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rovio_interfaces
bool
rovio_interfaces__srv__SrvResetToPose_Request__Sequence__are_equal(const rovio_interfaces__srv__SrvResetToPose_Request__Sequence * lhs, const rovio_interfaces__srv__SrvResetToPose_Request__Sequence * rhs);

/// Copy an array of srv/SrvResetToPose messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rovio_interfaces
bool
rovio_interfaces__srv__SrvResetToPose_Request__Sequence__copy(
  const rovio_interfaces__srv__SrvResetToPose_Request__Sequence * input,
  rovio_interfaces__srv__SrvResetToPose_Request__Sequence * output);

/// Initialize srv/SrvResetToPose message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rovio_interfaces__srv__SrvResetToPose_Response
 * )) before or use
 * rovio_interfaces__srv__SrvResetToPose_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rovio_interfaces
bool
rovio_interfaces__srv__SrvResetToPose_Response__init(rovio_interfaces__srv__SrvResetToPose_Response * msg);

/// Finalize srv/SrvResetToPose message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rovio_interfaces
void
rovio_interfaces__srv__SrvResetToPose_Response__fini(rovio_interfaces__srv__SrvResetToPose_Response * msg);

/// Create srv/SrvResetToPose message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rovio_interfaces__srv__SrvResetToPose_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rovio_interfaces
rovio_interfaces__srv__SrvResetToPose_Response *
rovio_interfaces__srv__SrvResetToPose_Response__create();

/// Destroy srv/SrvResetToPose message.
/**
 * It calls
 * rovio_interfaces__srv__SrvResetToPose_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rovio_interfaces
void
rovio_interfaces__srv__SrvResetToPose_Response__destroy(rovio_interfaces__srv__SrvResetToPose_Response * msg);

/// Check for srv/SrvResetToPose message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rovio_interfaces
bool
rovio_interfaces__srv__SrvResetToPose_Response__are_equal(const rovio_interfaces__srv__SrvResetToPose_Response * lhs, const rovio_interfaces__srv__SrvResetToPose_Response * rhs);

/// Copy a srv/SrvResetToPose message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rovio_interfaces
bool
rovio_interfaces__srv__SrvResetToPose_Response__copy(
  const rovio_interfaces__srv__SrvResetToPose_Response * input,
  rovio_interfaces__srv__SrvResetToPose_Response * output);

/// Initialize array of srv/SrvResetToPose messages.
/**
 * It allocates the memory for the number of elements and calls
 * rovio_interfaces__srv__SrvResetToPose_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rovio_interfaces
bool
rovio_interfaces__srv__SrvResetToPose_Response__Sequence__init(rovio_interfaces__srv__SrvResetToPose_Response__Sequence * array, size_t size);

/// Finalize array of srv/SrvResetToPose messages.
/**
 * It calls
 * rovio_interfaces__srv__SrvResetToPose_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rovio_interfaces
void
rovio_interfaces__srv__SrvResetToPose_Response__Sequence__fini(rovio_interfaces__srv__SrvResetToPose_Response__Sequence * array);

/// Create array of srv/SrvResetToPose messages.
/**
 * It allocates the memory for the array and calls
 * rovio_interfaces__srv__SrvResetToPose_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rovio_interfaces
rovio_interfaces__srv__SrvResetToPose_Response__Sequence *
rovio_interfaces__srv__SrvResetToPose_Response__Sequence__create(size_t size);

/// Destroy array of srv/SrvResetToPose messages.
/**
 * It calls
 * rovio_interfaces__srv__SrvResetToPose_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rovio_interfaces
void
rovio_interfaces__srv__SrvResetToPose_Response__Sequence__destroy(rovio_interfaces__srv__SrvResetToPose_Response__Sequence * array);

/// Check for srv/SrvResetToPose message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rovio_interfaces
bool
rovio_interfaces__srv__SrvResetToPose_Response__Sequence__are_equal(const rovio_interfaces__srv__SrvResetToPose_Response__Sequence * lhs, const rovio_interfaces__srv__SrvResetToPose_Response__Sequence * rhs);

/// Copy an array of srv/SrvResetToPose messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rovio_interfaces
bool
rovio_interfaces__srv__SrvResetToPose_Response__Sequence__copy(
  const rovio_interfaces__srv__SrvResetToPose_Response__Sequence * input,
  rovio_interfaces__srv__SrvResetToPose_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROVIO_INTERFACES__SRV__DETAIL__SRV_RESET_TO_POSE__FUNCTIONS_H_
