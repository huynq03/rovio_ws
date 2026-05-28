// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rovio_interfaces:msg/Health.idl
// generated code does not contain a copyright notice

#ifndef ROVIO_INTERFACES__MSG__DETAIL__HEALTH__TRAITS_HPP_
#define ROVIO_INTERFACES__MSG__DETAIL__HEALTH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rovio_interfaces/msg/detail/health__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rovio_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Health & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: valid_feature_ratio
  {
    out << "valid_feature_ratio: ";
    rosidl_generator_traits::value_to_yaml(msg.valid_feature_ratio, out);
    out << ", ";
  }

  // member: tracked_feature_ratio
  {
    out << "tracked_feature_ratio: ";
    rosidl_generator_traits::value_to_yaml(msg.tracked_feature_ratio, out);
    out << ", ";
  }

  // member: pixel_covariance_ratio
  {
    out << "pixel_covariance_ratio: ";
    rosidl_generator_traits::value_to_yaml(msg.pixel_covariance_ratio, out);
    out << ", ";
  }

  // member: nis_z_score_rmse
  {
    out << "nis_z_score_rmse: ";
    rosidl_generator_traits::value_to_yaml(msg.nis_z_score_rmse, out);
    out << ", ";
  }

  // member: accel_deviation
  {
    out << "accel_deviation: ";
    rosidl_generator_traits::value_to_yaml(msg.accel_deviation, out);
    out << ", ";
  }

  // member: speed_deviation
  {
    out << "speed_deviation: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_deviation, out);
    out << ", ";
  }

  // member: depth_feature_cov_median
  {
    out << "depth_feature_cov_median: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_feature_cov_median, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Health & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: valid_feature_ratio
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "valid_feature_ratio: ";
    rosidl_generator_traits::value_to_yaml(msg.valid_feature_ratio, out);
    out << "\n";
  }

  // member: tracked_feature_ratio
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tracked_feature_ratio: ";
    rosidl_generator_traits::value_to_yaml(msg.tracked_feature_ratio, out);
    out << "\n";
  }

  // member: pixel_covariance_ratio
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pixel_covariance_ratio: ";
    rosidl_generator_traits::value_to_yaml(msg.pixel_covariance_ratio, out);
    out << "\n";
  }

  // member: nis_z_score_rmse
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nis_z_score_rmse: ";
    rosidl_generator_traits::value_to_yaml(msg.nis_z_score_rmse, out);
    out << "\n";
  }

  // member: accel_deviation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accel_deviation: ";
    rosidl_generator_traits::value_to_yaml(msg.accel_deviation, out);
    out << "\n";
  }

  // member: speed_deviation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed_deviation: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_deviation, out);
    out << "\n";
  }

  // member: depth_feature_cov_median
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "depth_feature_cov_median: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_feature_cov_median, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Health & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace rovio_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use rovio_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rovio_interfaces::msg::Health & msg,
  std::ostream & out, size_t indentation = 0)
{
  rovio_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rovio_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const rovio_interfaces::msg::Health & msg)
{
  return rovio_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rovio_interfaces::msg::Health>()
{
  return "rovio_interfaces::msg::Health";
}

template<>
inline const char * name<rovio_interfaces::msg::Health>()
{
  return "rovio_interfaces/msg/Health";
}

template<>
struct has_fixed_size<rovio_interfaces::msg::Health>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<rovio_interfaces::msg::Health>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<rovio_interfaces::msg::Health>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROVIO_INTERFACES__MSG__DETAIL__HEALTH__TRAITS_HPP_
