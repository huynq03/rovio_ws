// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from rovio_interfaces:msg/Health.idl
// generated code does not contain a copyright notice
#include "rovio_interfaces/msg/detail/health__rosidl_typesupport_fastrtps_cpp.hpp"
#include "rovio_interfaces/msg/detail/health__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs


namespace rovio_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rovio_interfaces
cdr_serialize(
  const rovio_interfaces::msg::Health & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: valid_feature_ratio
  cdr << ros_message.valid_feature_ratio;
  // Member: tracked_feature_ratio
  cdr << ros_message.tracked_feature_ratio;
  // Member: pixel_covariance_ratio
  cdr << ros_message.pixel_covariance_ratio;
  // Member: nis_z_score_rmse
  cdr << ros_message.nis_z_score_rmse;
  // Member: accel_deviation
  cdr << ros_message.accel_deviation;
  // Member: speed_deviation
  cdr << ros_message.speed_deviation;
  // Member: depth_feature_cov_median
  cdr << ros_message.depth_feature_cov_median;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rovio_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  rovio_interfaces::msg::Health & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: valid_feature_ratio
  cdr >> ros_message.valid_feature_ratio;

  // Member: tracked_feature_ratio
  cdr >> ros_message.tracked_feature_ratio;

  // Member: pixel_covariance_ratio
  cdr >> ros_message.pixel_covariance_ratio;

  // Member: nis_z_score_rmse
  cdr >> ros_message.nis_z_score_rmse;

  // Member: accel_deviation
  cdr >> ros_message.accel_deviation;

  // Member: speed_deviation
  cdr >> ros_message.speed_deviation;

  // Member: depth_feature_cov_median
  cdr >> ros_message.depth_feature_cov_median;

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rovio_interfaces
get_serialized_size(
  const rovio_interfaces::msg::Health & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: valid_feature_ratio
  {
    size_t item_size = sizeof(ros_message.valid_feature_ratio);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: tracked_feature_ratio
  {
    size_t item_size = sizeof(ros_message.tracked_feature_ratio);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pixel_covariance_ratio
  {
    size_t item_size = sizeof(ros_message.pixel_covariance_ratio);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: nis_z_score_rmse
  {
    size_t item_size = sizeof(ros_message.nis_z_score_rmse);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: accel_deviation
  {
    size_t item_size = sizeof(ros_message.accel_deviation);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: speed_deviation
  {
    size_t item_size = sizeof(ros_message.speed_deviation);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: depth_feature_cov_median
  {
    size_t item_size = sizeof(ros_message.depth_feature_cov_median);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rovio_interfaces
max_serialized_size_Health(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: valid_feature_ratio
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: tracked_feature_ratio
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: pixel_covariance_ratio
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: nis_z_score_rmse
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: accel_deviation
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: speed_deviation
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: depth_feature_cov_median
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = rovio_interfaces::msg::Health;
    is_plain =
      (
      offsetof(DataType, depth_feature_cov_median) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _Health__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const rovio_interfaces::msg::Health *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Health__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<rovio_interfaces::msg::Health *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Health__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const rovio_interfaces::msg::Health *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Health__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Health(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Health__callbacks = {
  "rovio_interfaces::msg",
  "Health",
  _Health__cdr_serialize,
  _Health__cdr_deserialize,
  _Health__get_serialized_size,
  _Health__max_serialized_size
};

static rosidl_message_type_support_t _Health__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Health__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace rovio_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_rovio_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<rovio_interfaces::msg::Health>()
{
  return &rovio_interfaces::msg::typesupport_fastrtps_cpp::_Health__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rovio_interfaces, msg, Health)() {
  return &rovio_interfaces::msg::typesupport_fastrtps_cpp::_Health__handle;
}

#ifdef __cplusplus
}
#endif
