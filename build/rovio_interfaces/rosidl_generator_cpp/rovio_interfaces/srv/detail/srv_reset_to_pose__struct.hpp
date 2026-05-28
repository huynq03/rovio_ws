// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rovio_interfaces:srv/SrvResetToPose.idl
// generated code does not contain a copyright notice

#ifndef ROVIO_INTERFACES__SRV__DETAIL__SRV_RESET_TO_POSE__STRUCT_HPP_
#define ROVIO_INTERFACES__SRV__DETAIL__SRV_RESET_TO_POSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 't_wm'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rovio_interfaces__srv__SrvResetToPose_Request __attribute__((deprecated))
#else
# define DEPRECATED__rovio_interfaces__srv__SrvResetToPose_Request __declspec(deprecated)
#endif

namespace rovio_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SrvResetToPose_Request_
{
  using Type = SrvResetToPose_Request_<ContainerAllocator>;

  explicit SrvResetToPose_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : t_wm(_init)
  {
    (void)_init;
  }

  explicit SrvResetToPose_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : t_wm(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _t_wm_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _t_wm_type t_wm;

  // setters for named parameter idiom
  Type & set__t_wm(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->t_wm = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rovio_interfaces::srv::SrvResetToPose_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rovio_interfaces::srv::SrvResetToPose_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rovio_interfaces::srv::SrvResetToPose_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rovio_interfaces::srv::SrvResetToPose_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rovio_interfaces::srv::SrvResetToPose_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rovio_interfaces::srv::SrvResetToPose_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rovio_interfaces::srv::SrvResetToPose_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rovio_interfaces::srv::SrvResetToPose_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rovio_interfaces::srv::SrvResetToPose_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rovio_interfaces::srv::SrvResetToPose_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rovio_interfaces__srv__SrvResetToPose_Request
    std::shared_ptr<rovio_interfaces::srv::SrvResetToPose_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rovio_interfaces__srv__SrvResetToPose_Request
    std::shared_ptr<rovio_interfaces::srv::SrvResetToPose_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SrvResetToPose_Request_ & other) const
  {
    if (this->t_wm != other.t_wm) {
      return false;
    }
    return true;
  }
  bool operator!=(const SrvResetToPose_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SrvResetToPose_Request_

// alias to use template instance with default allocator
using SrvResetToPose_Request =
  rovio_interfaces::srv::SrvResetToPose_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rovio_interfaces


// Include directives for member types
// Member 'nothing'
#include "std_msgs/msg/detail/empty__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rovio_interfaces__srv__SrvResetToPose_Response __attribute__((deprecated))
#else
# define DEPRECATED__rovio_interfaces__srv__SrvResetToPose_Response __declspec(deprecated)
#endif

namespace rovio_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SrvResetToPose_Response_
{
  using Type = SrvResetToPose_Response_<ContainerAllocator>;

  explicit SrvResetToPose_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : nothing(_init)
  {
    (void)_init;
  }

  explicit SrvResetToPose_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : nothing(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _nothing_type =
    std_msgs::msg::Empty_<ContainerAllocator>;
  _nothing_type nothing;

  // setters for named parameter idiom
  Type & set__nothing(
    const std_msgs::msg::Empty_<ContainerAllocator> & _arg)
  {
    this->nothing = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rovio_interfaces::srv::SrvResetToPose_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rovio_interfaces::srv::SrvResetToPose_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rovio_interfaces::srv::SrvResetToPose_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rovio_interfaces::srv::SrvResetToPose_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rovio_interfaces::srv::SrvResetToPose_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rovio_interfaces::srv::SrvResetToPose_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rovio_interfaces::srv::SrvResetToPose_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rovio_interfaces::srv::SrvResetToPose_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rovio_interfaces::srv::SrvResetToPose_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rovio_interfaces::srv::SrvResetToPose_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rovio_interfaces__srv__SrvResetToPose_Response
    std::shared_ptr<rovio_interfaces::srv::SrvResetToPose_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rovio_interfaces__srv__SrvResetToPose_Response
    std::shared_ptr<rovio_interfaces::srv::SrvResetToPose_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SrvResetToPose_Response_ & other) const
  {
    if (this->nothing != other.nothing) {
      return false;
    }
    return true;
  }
  bool operator!=(const SrvResetToPose_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SrvResetToPose_Response_

// alias to use template instance with default allocator
using SrvResetToPose_Response =
  rovio_interfaces::srv::SrvResetToPose_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rovio_interfaces

namespace rovio_interfaces
{

namespace srv
{

struct SrvResetToPose
{
  using Request = rovio_interfaces::srv::SrvResetToPose_Request;
  using Response = rovio_interfaces::srv::SrvResetToPose_Response;
};

}  // namespace srv

}  // namespace rovio_interfaces

#endif  // ROVIO_INTERFACES__SRV__DETAIL__SRV_RESET_TO_POSE__STRUCT_HPP_
