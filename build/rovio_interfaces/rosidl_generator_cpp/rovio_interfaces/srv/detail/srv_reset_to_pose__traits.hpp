// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rovio_interfaces:srv/SrvResetToPose.idl
// generated code does not contain a copyright notice

#ifndef ROVIO_INTERFACES__SRV__DETAIL__SRV_RESET_TO_POSE__TRAITS_HPP_
#define ROVIO_INTERFACES__SRV__DETAIL__SRV_RESET_TO_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rovio_interfaces/srv/detail/srv_reset_to_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 't_wm'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace rovio_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SrvResetToPose_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: t_wm
  {
    out << "t_wm: ";
    to_flow_style_yaml(msg.t_wm, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SrvResetToPose_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: t_wm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "t_wm:\n";
    to_block_style_yaml(msg.t_wm, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SrvResetToPose_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace rovio_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use rovio_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rovio_interfaces::srv::SrvResetToPose_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rovio_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rovio_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const rovio_interfaces::srv::SrvResetToPose_Request & msg)
{
  return rovio_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rovio_interfaces::srv::SrvResetToPose_Request>()
{
  return "rovio_interfaces::srv::SrvResetToPose_Request";
}

template<>
inline const char * name<rovio_interfaces::srv::SrvResetToPose_Request>()
{
  return "rovio_interfaces/srv/SrvResetToPose_Request";
}

template<>
struct has_fixed_size<rovio_interfaces::srv::SrvResetToPose_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct has_bounded_size<rovio_interfaces::srv::SrvResetToPose_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct is_message<rovio_interfaces::srv::SrvResetToPose_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'nothing'
#include "std_msgs/msg/detail/empty__traits.hpp"

namespace rovio_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SrvResetToPose_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: nothing
  {
    out << "nothing: ";
    to_flow_style_yaml(msg.nothing, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SrvResetToPose_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: nothing
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nothing:\n";
    to_block_style_yaml(msg.nothing, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SrvResetToPose_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace rovio_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use rovio_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rovio_interfaces::srv::SrvResetToPose_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rovio_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rovio_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const rovio_interfaces::srv::SrvResetToPose_Response & msg)
{
  return rovio_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rovio_interfaces::srv::SrvResetToPose_Response>()
{
  return "rovio_interfaces::srv::SrvResetToPose_Response";
}

template<>
inline const char * name<rovio_interfaces::srv::SrvResetToPose_Response>()
{
  return "rovio_interfaces/srv/SrvResetToPose_Response";
}

template<>
struct has_fixed_size<rovio_interfaces::srv::SrvResetToPose_Response>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Empty>::value> {};

template<>
struct has_bounded_size<rovio_interfaces::srv::SrvResetToPose_Response>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Empty>::value> {};

template<>
struct is_message<rovio_interfaces::srv::SrvResetToPose_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rovio_interfaces::srv::SrvResetToPose>()
{
  return "rovio_interfaces::srv::SrvResetToPose";
}

template<>
inline const char * name<rovio_interfaces::srv::SrvResetToPose>()
{
  return "rovio_interfaces/srv/SrvResetToPose";
}

template<>
struct has_fixed_size<rovio_interfaces::srv::SrvResetToPose>
  : std::integral_constant<
    bool,
    has_fixed_size<rovio_interfaces::srv::SrvResetToPose_Request>::value &&
    has_fixed_size<rovio_interfaces::srv::SrvResetToPose_Response>::value
  >
{
};

template<>
struct has_bounded_size<rovio_interfaces::srv::SrvResetToPose>
  : std::integral_constant<
    bool,
    has_bounded_size<rovio_interfaces::srv::SrvResetToPose_Request>::value &&
    has_bounded_size<rovio_interfaces::srv::SrvResetToPose_Response>::value
  >
{
};

template<>
struct is_service<rovio_interfaces::srv::SrvResetToPose>
  : std::true_type
{
};

template<>
struct is_service_request<rovio_interfaces::srv::SrvResetToPose_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rovio_interfaces::srv::SrvResetToPose_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROVIO_INTERFACES__SRV__DETAIL__SRV_RESET_TO_POSE__TRAITS_HPP_
