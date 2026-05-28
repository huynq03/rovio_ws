#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to rovio_interfaces__srv__SrvResetToPose_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SrvResetToPose_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub t_wm: geometry_msgs::msg::Pose,

}



impl Default for SrvResetToPose_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SrvResetToPose_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SrvResetToPose_Request {
  type RmwMsg = super::srv::rmw::SrvResetToPose_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        t_wm: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(msg.t_wm)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        t_wm: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(&msg.t_wm)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      t_wm: geometry_msgs::msg::Pose::from_rmw_message(msg.t_wm),
    }
  }
}


// Corresponds to rovio_interfaces__srv__SrvResetToPose_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SrvResetToPose_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub nothing: std_msgs::msg::Empty,

}



impl Default for SrvResetToPose_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SrvResetToPose_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SrvResetToPose_Response {
  type RmwMsg = super::srv::rmw::SrvResetToPose_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        nothing: std_msgs::msg::Empty::into_rmw_message(std::borrow::Cow::Owned(msg.nothing)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        nothing: std_msgs::msg::Empty::into_rmw_message(std::borrow::Cow::Borrowed(&msg.nothing)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      nothing: std_msgs::msg::Empty::from_rmw_message(msg.nothing),
    }
  }
}






#[link(name = "rovio_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__rovio_interfaces__srv__SrvResetToPose() -> *const std::ffi::c_void;
}

// Corresponds to rovio_interfaces__srv__SrvResetToPose
#[allow(missing_docs, non_camel_case_types)]
pub struct SrvResetToPose;

impl rosidl_runtime_rs::Service for SrvResetToPose {
    type Request = SrvResetToPose_Request;
    type Response = SrvResetToPose_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__rovio_interfaces__srv__SrvResetToPose() }
    }
}


