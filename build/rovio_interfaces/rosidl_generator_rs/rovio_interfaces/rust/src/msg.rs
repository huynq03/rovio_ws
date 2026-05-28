#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to rovio_interfaces__msg__Health

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Health {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Health::default())
  }
}

impl rosidl_runtime_rs::Message for Health {
  type RmwMsg = super::msg::rmw::Health;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        valid_feature_ratio: msg.valid_feature_ratio,
        tracked_feature_ratio: msg.tracked_feature_ratio,
        pixel_covariance_ratio: msg.pixel_covariance_ratio,
        nis_z_score_rmse: msg.nis_z_score_rmse,
        accel_deviation: msg.accel_deviation,
        speed_deviation: msg.speed_deviation,
        depth_feature_cov_median: msg.depth_feature_cov_median,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      valid_feature_ratio: msg.valid_feature_ratio,
      tracked_feature_ratio: msg.tracked_feature_ratio,
      pixel_covariance_ratio: msg.pixel_covariance_ratio,
      nis_z_score_rmse: msg.nis_z_score_rmse,
      accel_deviation: msg.accel_deviation,
      speed_deviation: msg.speed_deviation,
      depth_feature_cov_median: msg.depth_feature_cov_median,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      valid_feature_ratio: msg.valid_feature_ratio,
      tracked_feature_ratio: msg.tracked_feature_ratio,
      pixel_covariance_ratio: msg.pixel_covariance_ratio,
      nis_z_score_rmse: msg.nis_z_score_rmse,
      accel_deviation: msg.accel_deviation,
      speed_deviation: msg.speed_deviation,
      depth_feature_cov_median: msg.depth_feature_cov_median,
    }
  }
}


