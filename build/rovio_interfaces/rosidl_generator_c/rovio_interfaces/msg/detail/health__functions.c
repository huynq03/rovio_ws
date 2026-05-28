// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rovio_interfaces:msg/Health.idl
// generated code does not contain a copyright notice
#include "rovio_interfaces/msg/detail/health__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
rovio_interfaces__msg__Health__init(rovio_interfaces__msg__Health * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    rovio_interfaces__msg__Health__fini(msg);
    return false;
  }
  // valid_feature_ratio
  // tracked_feature_ratio
  // pixel_covariance_ratio
  // nis_z_score_rmse
  // accel_deviation
  // speed_deviation
  // depth_feature_cov_median
  return true;
}

void
rovio_interfaces__msg__Health__fini(rovio_interfaces__msg__Health * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // valid_feature_ratio
  // tracked_feature_ratio
  // pixel_covariance_ratio
  // nis_z_score_rmse
  // accel_deviation
  // speed_deviation
  // depth_feature_cov_median
}

bool
rovio_interfaces__msg__Health__are_equal(const rovio_interfaces__msg__Health * lhs, const rovio_interfaces__msg__Health * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // valid_feature_ratio
  if (lhs->valid_feature_ratio != rhs->valid_feature_ratio) {
    return false;
  }
  // tracked_feature_ratio
  if (lhs->tracked_feature_ratio != rhs->tracked_feature_ratio) {
    return false;
  }
  // pixel_covariance_ratio
  if (lhs->pixel_covariance_ratio != rhs->pixel_covariance_ratio) {
    return false;
  }
  // nis_z_score_rmse
  if (lhs->nis_z_score_rmse != rhs->nis_z_score_rmse) {
    return false;
  }
  // accel_deviation
  if (lhs->accel_deviation != rhs->accel_deviation) {
    return false;
  }
  // speed_deviation
  if (lhs->speed_deviation != rhs->speed_deviation) {
    return false;
  }
  // depth_feature_cov_median
  if (lhs->depth_feature_cov_median != rhs->depth_feature_cov_median) {
    return false;
  }
  return true;
}

bool
rovio_interfaces__msg__Health__copy(
  const rovio_interfaces__msg__Health * input,
  rovio_interfaces__msg__Health * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // valid_feature_ratio
  output->valid_feature_ratio = input->valid_feature_ratio;
  // tracked_feature_ratio
  output->tracked_feature_ratio = input->tracked_feature_ratio;
  // pixel_covariance_ratio
  output->pixel_covariance_ratio = input->pixel_covariance_ratio;
  // nis_z_score_rmse
  output->nis_z_score_rmse = input->nis_z_score_rmse;
  // accel_deviation
  output->accel_deviation = input->accel_deviation;
  // speed_deviation
  output->speed_deviation = input->speed_deviation;
  // depth_feature_cov_median
  output->depth_feature_cov_median = input->depth_feature_cov_median;
  return true;
}

rovio_interfaces__msg__Health *
rovio_interfaces__msg__Health__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rovio_interfaces__msg__Health * msg = (rovio_interfaces__msg__Health *)allocator.allocate(sizeof(rovio_interfaces__msg__Health), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rovio_interfaces__msg__Health));
  bool success = rovio_interfaces__msg__Health__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rovio_interfaces__msg__Health__destroy(rovio_interfaces__msg__Health * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rovio_interfaces__msg__Health__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rovio_interfaces__msg__Health__Sequence__init(rovio_interfaces__msg__Health__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rovio_interfaces__msg__Health * data = NULL;

  if (size) {
    data = (rovio_interfaces__msg__Health *)allocator.zero_allocate(size, sizeof(rovio_interfaces__msg__Health), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rovio_interfaces__msg__Health__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rovio_interfaces__msg__Health__fini(&data[i - 1]);
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
rovio_interfaces__msg__Health__Sequence__fini(rovio_interfaces__msg__Health__Sequence * array)
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
      rovio_interfaces__msg__Health__fini(&array->data[i]);
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

rovio_interfaces__msg__Health__Sequence *
rovio_interfaces__msg__Health__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rovio_interfaces__msg__Health__Sequence * array = (rovio_interfaces__msg__Health__Sequence *)allocator.allocate(sizeof(rovio_interfaces__msg__Health__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rovio_interfaces__msg__Health__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rovio_interfaces__msg__Health__Sequence__destroy(rovio_interfaces__msg__Health__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rovio_interfaces__msg__Health__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rovio_interfaces__msg__Health__Sequence__are_equal(const rovio_interfaces__msg__Health__Sequence * lhs, const rovio_interfaces__msg__Health__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rovio_interfaces__msg__Health__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rovio_interfaces__msg__Health__Sequence__copy(
  const rovio_interfaces__msg__Health__Sequence * input,
  rovio_interfaces__msg__Health__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rovio_interfaces__msg__Health);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rovio_interfaces__msg__Health * data =
      (rovio_interfaces__msg__Health *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rovio_interfaces__msg__Health__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rovio_interfaces__msg__Health__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rovio_interfaces__msg__Health__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
