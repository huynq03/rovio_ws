// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rovio_interfaces:msg/Health.idl
// generated code does not contain a copyright notice

#ifndef ROVIO_INTERFACES__MSG__DETAIL__HEALTH__STRUCT_HPP_
#define ROVIO_INTERFACES__MSG__DETAIL__HEALTH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rovio_interfaces__msg__Health __attribute__((deprecated))
#else
# define DEPRECATED__rovio_interfaces__msg__Health __declspec(deprecated)
#endif

namespace rovio_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Health_
{
  using Type = Health_<ContainerAllocator>;

  explicit Health_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->valid_feature_ratio = 0.0f;
      this->tracked_feature_ratio = 0.0f;
      this->pixel_covariance_ratio = 0.0f;
      this->nis_z_score_rmse = 0.0f;
      this->accel_deviation = 0.0f;
      this->speed_deviation = 0.0f;
      this->depth_feature_cov_median = 0.0f;
    }
  }

  explicit Health_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->valid_feature_ratio = 0.0f;
      this->tracked_feature_ratio = 0.0f;
      this->pixel_covariance_ratio = 0.0f;
      this->nis_z_score_rmse = 0.0f;
      this->accel_deviation = 0.0f;
      this->speed_deviation = 0.0f;
      this->depth_feature_cov_median = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _valid_feature_ratio_type =
    float;
  _valid_feature_ratio_type valid_feature_ratio;
  using _tracked_feature_ratio_type =
    float;
  _tracked_feature_ratio_type tracked_feature_ratio;
  using _pixel_covariance_ratio_type =
    float;
  _pixel_covariance_ratio_type pixel_covariance_ratio;
  using _nis_z_score_rmse_type =
    float;
  _nis_z_score_rmse_type nis_z_score_rmse;
  using _accel_deviation_type =
    float;
  _accel_deviation_type accel_deviation;
  using _speed_deviation_type =
    float;
  _speed_deviation_type speed_deviation;
  using _depth_feature_cov_median_type =
    float;
  _depth_feature_cov_median_type depth_feature_cov_median;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__valid_feature_ratio(
    const float & _arg)
  {
    this->valid_feature_ratio = _arg;
    return *this;
  }
  Type & set__tracked_feature_ratio(
    const float & _arg)
  {
    this->tracked_feature_ratio = _arg;
    return *this;
  }
  Type & set__pixel_covariance_ratio(
    const float & _arg)
  {
    this->pixel_covariance_ratio = _arg;
    return *this;
  }
  Type & set__nis_z_score_rmse(
    const float & _arg)
  {
    this->nis_z_score_rmse = _arg;
    return *this;
  }
  Type & set__accel_deviation(
    const float & _arg)
  {
    this->accel_deviation = _arg;
    return *this;
  }
  Type & set__speed_deviation(
    const float & _arg)
  {
    this->speed_deviation = _arg;
    return *this;
  }
  Type & set__depth_feature_cov_median(
    const float & _arg)
  {
    this->depth_feature_cov_median = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rovio_interfaces::msg::Health_<ContainerAllocator> *;
  using ConstRawPtr =
    const rovio_interfaces::msg::Health_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rovio_interfaces::msg::Health_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rovio_interfaces::msg::Health_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rovio_interfaces::msg::Health_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rovio_interfaces::msg::Health_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rovio_interfaces::msg::Health_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rovio_interfaces::msg::Health_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rovio_interfaces::msg::Health_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rovio_interfaces::msg::Health_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rovio_interfaces__msg__Health
    std::shared_ptr<rovio_interfaces::msg::Health_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rovio_interfaces__msg__Health
    std::shared_ptr<rovio_interfaces::msg::Health_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Health_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->valid_feature_ratio != other.valid_feature_ratio) {
      return false;
    }
    if (this->tracked_feature_ratio != other.tracked_feature_ratio) {
      return false;
    }
    if (this->pixel_covariance_ratio != other.pixel_covariance_ratio) {
      return false;
    }
    if (this->nis_z_score_rmse != other.nis_z_score_rmse) {
      return false;
    }
    if (this->accel_deviation != other.accel_deviation) {
      return false;
    }
    if (this->speed_deviation != other.speed_deviation) {
      return false;
    }
    if (this->depth_feature_cov_median != other.depth_feature_cov_median) {
      return false;
    }
    return true;
  }
  bool operator!=(const Health_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Health_

// alias to use template instance with default allocator
using Health =
  rovio_interfaces::msg::Health_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rovio_interfaces

#endif  // ROVIO_INTERFACES__MSG__DETAIL__HEALTH__STRUCT_HPP_
