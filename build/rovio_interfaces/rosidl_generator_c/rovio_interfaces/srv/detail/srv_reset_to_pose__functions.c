// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rovio_interfaces:srv/SrvResetToPose.idl
// generated code does not contain a copyright notice
#include "rovio_interfaces/srv/detail/srv_reset_to_pose__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `t_wm`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
rovio_interfaces__srv__SrvResetToPose_Request__init(rovio_interfaces__srv__SrvResetToPose_Request * msg)
{
  if (!msg) {
    return false;
  }
  // t_wm
  if (!geometry_msgs__msg__Pose__init(&msg->t_wm)) {
    rovio_interfaces__srv__SrvResetToPose_Request__fini(msg);
    return false;
  }
  return true;
}

void
rovio_interfaces__srv__SrvResetToPose_Request__fini(rovio_interfaces__srv__SrvResetToPose_Request * msg)
{
  if (!msg) {
    return;
  }
  // t_wm
  geometry_msgs__msg__Pose__fini(&msg->t_wm);
}

bool
rovio_interfaces__srv__SrvResetToPose_Request__are_equal(const rovio_interfaces__srv__SrvResetToPose_Request * lhs, const rovio_interfaces__srv__SrvResetToPose_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // t_wm
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->t_wm), &(rhs->t_wm)))
  {
    return false;
  }
  return true;
}

bool
rovio_interfaces__srv__SrvResetToPose_Request__copy(
  const rovio_interfaces__srv__SrvResetToPose_Request * input,
  rovio_interfaces__srv__SrvResetToPose_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // t_wm
  if (!geometry_msgs__msg__Pose__copy(
      &(input->t_wm), &(output->t_wm)))
  {
    return false;
  }
  return true;
}

rovio_interfaces__srv__SrvResetToPose_Request *
rovio_interfaces__srv__SrvResetToPose_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rovio_interfaces__srv__SrvResetToPose_Request * msg = (rovio_interfaces__srv__SrvResetToPose_Request *)allocator.allocate(sizeof(rovio_interfaces__srv__SrvResetToPose_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rovio_interfaces__srv__SrvResetToPose_Request));
  bool success = rovio_interfaces__srv__SrvResetToPose_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rovio_interfaces__srv__SrvResetToPose_Request__destroy(rovio_interfaces__srv__SrvResetToPose_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rovio_interfaces__srv__SrvResetToPose_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rovio_interfaces__srv__SrvResetToPose_Request__Sequence__init(rovio_interfaces__srv__SrvResetToPose_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rovio_interfaces__srv__SrvResetToPose_Request * data = NULL;

  if (size) {
    data = (rovio_interfaces__srv__SrvResetToPose_Request *)allocator.zero_allocate(size, sizeof(rovio_interfaces__srv__SrvResetToPose_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rovio_interfaces__srv__SrvResetToPose_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rovio_interfaces__srv__SrvResetToPose_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rovio_interfaces__srv__SrvResetToPose_Request__Sequence__fini(rovio_interfaces__srv__SrvResetToPose_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rovio_interfaces__srv__SrvResetToPose_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rovio_interfaces__srv__SrvResetToPose_Request__Sequence *
rovio_interfaces__srv__SrvResetToPose_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rovio_interfaces__srv__SrvResetToPose_Request__Sequence * array = (rovio_interfaces__srv__SrvResetToPose_Request__Sequence *)allocator.allocate(sizeof(rovio_interfaces__srv__SrvResetToPose_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rovio_interfaces__srv__SrvResetToPose_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rovio_interfaces__srv__SrvResetToPose_Request__Sequence__destroy(rovio_interfaces__srv__SrvResetToPose_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rovio_interfaces__srv__SrvResetToPose_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rovio_interfaces__srv__SrvResetToPose_Request__Sequence__are_equal(const rovio_interfaces__srv__SrvResetToPose_Request__Sequence * lhs, const rovio_interfaces__srv__SrvResetToPose_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rovio_interfaces__srv__SrvResetToPose_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rovio_interfaces__srv__SrvResetToPose_Request__Sequence__copy(
  const rovio_interfaces__srv__SrvResetToPose_Request__Sequence * input,
  rovio_interfaces__srv__SrvResetToPose_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rovio_interfaces__srv__SrvResetToPose_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rovio_interfaces__srv__SrvResetToPose_Request * data =
      (rovio_interfaces__srv__SrvResetToPose_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rovio_interfaces__srv__SrvResetToPose_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rovio_interfaces__srv__SrvResetToPose_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rovio_interfaces__srv__SrvResetToPose_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `nothing`
#include "std_msgs/msg/detail/empty__functions.h"

bool
rovio_interfaces__srv__SrvResetToPose_Response__init(rovio_interfaces__srv__SrvResetToPose_Response * msg)
{
  if (!msg) {
    return false;
  }
  // nothing
  if (!std_msgs__msg__Empty__init(&msg->nothing)) {
    rovio_interfaces__srv__SrvResetToPose_Response__fini(msg);
    return false;
  }
  return true;
}

void
rovio_interfaces__srv__SrvResetToPose_Response__fini(rovio_interfaces__srv__SrvResetToPose_Response * msg)
{
  if (!msg) {
    return;
  }
  // nothing
  std_msgs__msg__Empty__fini(&msg->nothing);
}

bool
rovio_interfaces__srv__SrvResetToPose_Response__are_equal(const rovio_interfaces__srv__SrvResetToPose_Response * lhs, const rovio_interfaces__srv__SrvResetToPose_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // nothing
  if (!std_msgs__msg__Empty__are_equal(
      &(lhs->nothing), &(rhs->nothing)))
  {
    return false;
  }
  return true;
}

bool
rovio_interfaces__srv__SrvResetToPose_Response__copy(
  const rovio_interfaces__srv__SrvResetToPose_Response * input,
  rovio_interfaces__srv__SrvResetToPose_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // nothing
  if (!std_msgs__msg__Empty__copy(
      &(input->nothing), &(output->nothing)))
  {
    return false;
  }
  return true;
}

rovio_interfaces__srv__SrvResetToPose_Response *
rovio_interfaces__srv__SrvResetToPose_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rovio_interfaces__srv__SrvResetToPose_Response * msg = (rovio_interfaces__srv__SrvResetToPose_Response *)allocator.allocate(sizeof(rovio_interfaces__srv__SrvResetToPose_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rovio_interfaces__srv__SrvResetToPose_Response));
  bool success = rovio_interfaces__srv__SrvResetToPose_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rovio_interfaces__srv__SrvResetToPose_Response__destroy(rovio_interfaces__srv__SrvResetToPose_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rovio_interfaces__srv__SrvResetToPose_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rovio_interfaces__srv__SrvResetToPose_Response__Sequence__init(rovio_interfaces__srv__SrvResetToPose_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rovio_interfaces__srv__SrvResetToPose_Response * data = NULL;

  if (size) {
    data = (rovio_interfaces__srv__SrvResetToPose_Response *)allocator.zero_allocate(size, sizeof(rovio_interfaces__srv__SrvResetToPose_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rovio_interfaces__srv__SrvResetToPose_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rovio_interfaces__srv__SrvResetToPose_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rovio_interfaces__srv__SrvResetToPose_Response__Sequence__fini(rovio_interfaces__srv__SrvResetToPose_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rovio_interfaces__srv__SrvResetToPose_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rovio_interfaces__srv__SrvResetToPose_Response__Sequence *
rovio_interfaces__srv__SrvResetToPose_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rovio_interfaces__srv__SrvResetToPose_Response__Sequence * array = (rovio_interfaces__srv__SrvResetToPose_Response__Sequence *)allocator.allocate(sizeof(rovio_interfaces__srv__SrvResetToPose_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rovio_interfaces__srv__SrvResetToPose_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rovio_interfaces__srv__SrvResetToPose_Response__Sequence__destroy(rovio_interfaces__srv__SrvResetToPose_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rovio_interfaces__srv__SrvResetToPose_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rovio_interfaces__srv__SrvResetToPose_Response__Sequence__are_equal(const rovio_interfaces__srv__SrvResetToPose_Response__Sequence * lhs, const rovio_interfaces__srv__SrvResetToPose_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rovio_interfaces__srv__SrvResetToPose_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rovio_interfaces__srv__SrvResetToPose_Response__Sequence__copy(
  const rovio_interfaces__srv__SrvResetToPose_Response__Sequence * input,
  rovio_interfaces__srv__SrvResetToPose_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rovio_interfaces__srv__SrvResetToPose_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rovio_interfaces__srv__SrvResetToPose_Response * data =
      (rovio_interfaces__srv__SrvResetToPose_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rovio_interfaces__srv__SrvResetToPose_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rovio_interfaces__srv__SrvResetToPose_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rovio_interfaces__srv__SrvResetToPose_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
