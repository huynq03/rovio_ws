#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "rovio_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rovio_interfaces__msg__Health() -> *const std::ffi::c_void;
}

#[link(name = "rovio_interfaces__rosidl_generator_c")]
extern "C" {
    fn rovio_interfaces__msg__Health__init(msg: *mut Health) -> bool;
    fn rovio_interfaces__msg__Health__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Health>, size: usize) -> bool;
    fn rovio_interfaces__msg__Health__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Health>);
    fn rovio_interfaces__msg__Health__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Health>, out_seq: *mut rosidl_runtime_rs::Sequence<Health>) -> bool;
}

// Corresponds to rovio_interfaces__msg__Health
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Health {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub valid_feature_ratio: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub tracked_feature_ratio: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pixel_covariance_ratio: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub nis_z_score_rmse: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub accel_deviation: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub speed_deviation: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub depth_feature_cov_median: f32,

}



impl Default for Health {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rovio_interfaces__msg__Health__init(&mut msg as *mut _) {
        panic!("Call to rovio_interfaces__msg__Health__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Health {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rovio_interfaces__msg__Health__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rovio_interfaces__msg__Health__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rovio_interfaces__msg__Health__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Health {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Health where Self: Sized {
  const TYPE_NAME: &'static str = "rovio_interfaces/msg/Health";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rovio_interfaces__msg__Health() }
  }
}


