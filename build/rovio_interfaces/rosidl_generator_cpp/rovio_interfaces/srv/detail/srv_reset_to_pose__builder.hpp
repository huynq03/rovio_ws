// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rovio_interfaces:srv/SrvResetToPose.idl
// generated code does not contain a copyright notice

#ifndef ROVIO_INTERFACES__SRV__DETAIL__SRV_RESET_TO_POSE__BUILDER_HPP_
#define ROVIO_INTERFACES__SRV__DETAIL__SRV_RESET_TO_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rovio_interfaces/srv/detail/srv_reset_to_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rovio_interfaces
{

namespace srv
{

namespace builder
{

class Init_SrvResetToPose_Request_t_wm
{
public:
  Init_SrvResetToPose_Request_t_wm()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rovio_interfaces::srv::SrvResetToPose_Request t_wm(::rovio_interfaces::srv::SrvResetToPose_Request::_t_wm_type arg)
  {
    msg_.t_wm = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rovio_interfaces::srv::SrvResetToPose_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rovio_interfaces::srv::SrvResetToPose_Request>()
{
  return rovio_interfaces::srv::builder::Init_SrvResetToPose_Request_t_wm();
}

}  // namespace rovio_interfaces


namespace rovio_interfaces
{

namespace srv
{

namespace builder
{

class Init_SrvResetToPose_Response_nothing
{
public:
  Init_SrvResetToPose_Response_nothing()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rovio_interfaces::srv::SrvResetToPose_Response nothing(::rovio_interfaces::srv::SrvResetToPose_Response::_nothing_type arg)
  {
    msg_.nothing = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rovio_interfaces::srv::SrvResetToPose_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rovio_interfaces::srv::SrvResetToPose_Response>()
{
  return rovio_interfaces::srv::builder::Init_SrvResetToPose_Response_nothing();
}

}  // namespace rovio_interfaces

#endif  // ROVIO_INTERFACES__SRV__DETAIL__SRV_RESET_TO_POSE__BUILDER_HPP_
