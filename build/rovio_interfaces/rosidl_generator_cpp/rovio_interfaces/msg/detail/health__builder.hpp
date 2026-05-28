// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rovio_interfaces:msg/Health.idl
// generated code does not contain a copyright notice

#ifndef ROVIO_INTERFACES__MSG__DETAIL__HEALTH__BUILDER_HPP_
#define ROVIO_INTERFACES__MSG__DETAIL__HEALTH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rovio_interfaces/msg/detail/health__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rovio_interfaces
{

namespace msg
{

namespace builder
{

class Init_Health_depth_feature_cov_median
{
public:
  explicit Init_Health_depth_feature_cov_median(::rovio_interfaces::msg::Health & msg)
  : msg_(msg)
  {}
  ::rovio_interfaces::msg::Health depth_feature_cov_median(::rovio_interfaces::msg::Health::_depth_feature_cov_median_type arg)
  {
    msg_.depth_feature_cov_median = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rovio_interfaces::msg::Health msg_;
};

class Init_Health_speed_deviation
{
public:
  explicit Init_Health_speed_deviation(::rovio_interfaces::msg::Health & msg)
  : msg_(msg)
  {}
  Init_Health_depth_feature_cov_median speed_deviation(::rovio_interfaces::msg::Health::_speed_deviation_type arg)
  {
    msg_.speed_deviation = std::move(arg);
    return Init_Health_depth_feature_cov_median(msg_);
  }

private:
  ::rovio_interfaces::msg::Health msg_;
};

class Init_Health_accel_deviation
{
public:
  explicit Init_Health_accel_deviation(::rovio_interfaces::msg::Health & msg)
  : msg_(msg)
  {}
  Init_Health_speed_deviation accel_deviation(::rovio_interfaces::msg::Health::_accel_deviation_type arg)
  {
    msg_.accel_deviation = std::move(arg);
    return Init_Health_speed_deviation(msg_);
  }

private:
  ::rovio_interfaces::msg::Health msg_;
};

class Init_Health_nis_z_score_rmse
{
public:
  explicit Init_Health_nis_z_score_rmse(::rovio_interfaces::msg::Health & msg)
  : msg_(msg)
  {}
  Init_Health_accel_deviation nis_z_score_rmse(::rovio_interfaces::msg::Health::_nis_z_score_rmse_type arg)
  {
    msg_.nis_z_score_rmse = std::move(arg);
    return Init_Health_accel_deviation(msg_);
  }

private:
  ::rovio_interfaces::msg::Health msg_;
};

class Init_Health_pixel_covariance_ratio
{
public:
  explicit Init_Health_pixel_covariance_ratio(::rovio_interfaces::msg::Health & msg)
  : msg_(msg)
  {}
  Init_Health_nis_z_score_rmse pixel_covariance_ratio(::rovio_interfaces::msg::Health::_pixel_covariance_ratio_type arg)
  {
    msg_.pixel_covariance_ratio = std::move(arg);
    return Init_Health_nis_z_score_rmse(msg_);
  }

private:
  ::rovio_interfaces::msg::Health msg_;
};

class Init_Health_tracked_feature_ratio
{
public:
  explicit Init_Health_tracked_feature_ratio(::rovio_interfaces::msg::Health & msg)
  : msg_(msg)
  {}
  Init_Health_pixel_covariance_ratio tracked_feature_ratio(::rovio_interfaces::msg::Health::_tracked_feature_ratio_type arg)
  {
    msg_.tracked_feature_ratio = std::move(arg);
    return Init_Health_pixel_covariance_ratio(msg_);
  }

private:
  ::rovio_interfaces::msg::Health msg_;
};

class Init_Health_valid_feature_ratio
{
public:
  explicit Init_Health_valid_feature_ratio(::rovio_interfaces::msg::Health & msg)
  : msg_(msg)
  {}
  Init_Health_tracked_feature_ratio valid_feature_ratio(::rovio_interfaces::msg::Health::_valid_feature_ratio_type arg)
  {
    msg_.valid_feature_ratio = std::move(arg);
    return Init_Health_tracked_feature_ratio(msg_);
  }

private:
  ::rovio_interfaces::msg::Health msg_;
};

class Init_Health_header
{
public:
  Init_Health_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Health_valid_feature_ratio header(::rovio_interfaces::msg::Health::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Health_valid_feature_ratio(msg_);
  }

private:
  ::rovio_interfaces::msg::Health msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rovio_interfaces::msg::Health>()
{
  return rovio_interfaces::msg::builder::Init_Health_header();
}

}  // namespace rovio_interfaces

#endif  // ROVIO_INTERFACES__MSG__DETAIL__HEALTH__BUILDER_HPP_
