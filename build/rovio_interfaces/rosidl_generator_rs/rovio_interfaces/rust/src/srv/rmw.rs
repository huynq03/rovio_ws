#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "rovio_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rovio_interfaces__srv__SrvResetToPose_Request() -> *const std::ffi::c_void;
}

#[link(name = "rovio_interfaces__rosidl_generator_c")]
extern "C" {
    fn rovio_interfaces__srv__SrvResetToPose_Request__init(msg: *mut SrvResetToPose_Request) -> bool;
    fn rovio_interfaces__srv__SrvResetToPose_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SrvResetToPose_Request>, size: usize) -> bool;
    fn rovio_interfaces__srv__SrvResetToPose_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SrvResetToPose_Request>);
    fn rovio_interfaces__srv__SrvResetToPose_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SrvResetToPose_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SrvResetToPose_Request>) -> bool;
}

// Corresponds to rovio_interfaces__srv__SrvResetToPose_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SrvResetToPose_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub t_wm: geometry_msgs::msg::rmw::Pose,

}



impl Default for SrvResetToPose_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rovio_interfaces__srv__SrvResetToPose_Request__init(&mut msg as *mut _) {
        panic!("Call to rovio_interfaces__srv__SrvResetToPose_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SrvResetToPose_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rovio_interfaces__srv__SrvResetToPose_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rovio_interfaces__srv__SrvResetToPose_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rovio_interfaces__srv__SrvResetToPose_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SrvResetToPose_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SrvResetToPose_Request where Self: Sized {
  const TYPE_NAME: &'static str = "rovio_interfaces/srv/SrvResetToPose_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rovio_interfaces__srv__SrvResetToPose_Request() }
  }
}


#[link(name = "rovio_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rovio_interfaces__srv__SrvResetToPose_Response() -> *const std::ffi::c_void;
}

#[link(name = "rovio_interfaces__rosidl_generator_c")]
extern "C" {
    fn rovio_interfaces__srv__SrvResetToPose_Response__init(msg: *mut SrvResetToPose_Response) -> bool;
    fn rovio_interfaces__srv__SrvResetToPose_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SrvResetToPose_Response>, size: usize) -> bool;
    fn rovio_interfaces__srv__SrvResetToPose_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SrvResetToPose_Response>);
    fn rovio_interfaces__srv__SrvResetToPose_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SrvResetToPose_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SrvResetToPose_Response>) -> bool;
}

// Corresponds to rovio_interfaces__srv__SrvResetToPose_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SrvResetToPose_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub nothing: std_msgs::msg::rmw::Empty,

}



impl Default for SrvResetToPose_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rovio_interfaces__srv__SrvResetToPose_Response__init(&mut msg as *mut _) {
        panic!("Call to rovio_interfaces__srv__SrvResetToPose_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SrvResetToPose_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rovio_interfaces__srv__SrvResetToPose_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rovio_interfaces__srv__SrvResetToPose_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rovio_interfaces__srv__SrvResetToPose_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SrvResetToPose_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SrvResetToPose_Response where Self: Sized {
  const TYPE_NAME: &'static str = "rovio_interfaces/srv/SrvResetToPose_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rovio_interfaces__srv__SrvResetToPose_Response() }
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


