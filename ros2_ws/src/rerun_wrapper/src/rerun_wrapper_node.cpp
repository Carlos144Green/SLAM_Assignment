/* =====================================================================
 * MIT License
 * 
 * Copyright (c) 2026 Omni Instrument Inc.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * ===================================================================== 
*/

#include "rerun_wrapper/rerun_wrapper_node.hpp"

#include "rerun_wrapper/rerun_bridge.hpp"

#include "rclcpp_components/register_node_macro.hpp"

#include <functional>
#include <memory>
#include <string>

namespace rerun_wrapper
{

namespace
{

constexpr double kSyncedFrameMaxIntervalSeconds = 0.05;

}  // namespace

RerunWrapperNode::RerunWrapperNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("rerun_wrapper_node", options)
{
  const std::string blueprint_path =
    this->declare_parameter<std::string>("rerun_blueprint_path", "");
  rerun_bridge_ = std::make_unique<RerunBridge>(blueprint_path);

  camera_info_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  sync_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  imu_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  odometry_error_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  tf_static_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions camera_info_options;
  camera_info_options.callback_group = camera_info_callback_group_;

  rclcpp::SubscriptionOptions imu_options;
  imu_options.callback_group = imu_callback_group_;

  rclcpp::SubscriptionOptions odometry_error_options;
  odometry_error_options.callback_group = odometry_error_callback_group_;

  rclcpp::SubscriptionOptions tf_static_options;
  tf_static_options.callback_group = tf_static_callback_group_;

  rclcpp::SubscriptionOptions sync_options;
  sync_options.callback_group = sync_callback_group_;

  const rclcpp::QoS qos{500U};
  rclcpp::QoS tf_static_qos{100U};
  tf_static_qos.transient_local().reliable();

  left_camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "left/camera_info",
    qos,
    std::bind(&RerunWrapperNode::handle_left_camera_info, this, std::placeholders::_1),
    camera_info_options);
  right_camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "right/camera_info",
    qos,
    std::bind(&RerunWrapperNode::handle_right_camera_info, this, std::placeholders::_1),
    camera_info_options);
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "imu",
    qos,
    std::bind(&RerunWrapperNode::handle_imu, this, std::placeholders::_1),
    imu_options);
  odometry_error_sub_ = this->create_subscription<stereo_vio_msgs::msg::OdometryError>(
    "stereo_vio/errors",
    qos,
    std::bind(&RerunWrapperNode::handle_odometry_error, this, std::placeholders::_1),
    odometry_error_options);
  tf_static_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
    "tf_static",
    tf_static_qos,
    std::bind(&RerunWrapperNode::handle_tf_static, this, std::placeholders::_1),
    tf_static_options);

  left_image_sub_.subscribe(this, "left/image_rect", qos.get_rmw_qos_profile(), sync_options);
  right_image_sub_.subscribe(this, "right/image_rect", qos.get_rmw_qos_profile(), sync_options);
  ground_truth_odom_sub_.subscribe(
    this,
    "ground_truth/odom",
    qos.get_rmw_qos_profile(),
    sync_options);
  stereo_vio_odom_sub_.subscribe(
    this,
    "stereo_vio/odom",
    qos.get_rmw_qos_profile(),
    sync_options);
  stereo_vio_observations_sub_.subscribe(
    this,
    "stereo_vio/observations",
    qos.get_rmw_qos_profile(),
    sync_options);

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::Image,
    nav_msgs::msg::Odometry,
    nav_msgs::msg::Odometry,
    stereo_vio_msgs::msg::StereoObservations>;
  synced_frame_sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(500U),
    left_image_sub_,
    right_image_sub_,
    ground_truth_odom_sub_,
    stereo_vio_odom_sub_,
    stereo_vio_observations_sub_);
  synced_frame_sync_->setMaxIntervalDuration(
    rclcpp::Duration::from_seconds(kSyncedFrameMaxIntervalSeconds));
  synced_frame_sync_->registerCallback(
    std::bind(
      &RerunWrapperNode::handle_synced_frame,
      this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3,
      std::placeholders::_4,
      std::placeholders::_5));
}

RerunWrapperNode::~RerunWrapperNode() = default;

void RerunWrapperNode::handle_left_camera_info(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  left_camera_info_ = *msg;
  rerun_bridge_->set_time(msg->header.stamp);
  rerun_bridge_->log_left_camera_info_once(left_camera_info_.value());
  left_camera_info_sub_.reset();
}

void RerunWrapperNode::handle_right_camera_info(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  right_camera_info_ = *msg;
  rerun_bridge_->set_time(msg->header.stamp);
  rerun_bridge_->log_right_camera_info_once(right_camera_info_.value());
  right_camera_info_sub_.reset();
}

void RerunWrapperNode::handle_imu(const sensor_msgs::msg::Imu::ConstSharedPtr & msg)
{
  rerun_bridge_->log_imu(*msg);
}

void RerunWrapperNode::handle_odometry_error(
  const stereo_vio_msgs::msg::OdometryError::ConstSharedPtr & msg)
{
  rerun_bridge_->log_odometry_error(*msg);
}

void RerunWrapperNode::handle_tf_static(const tf2_msgs::msg::TFMessage::ConstSharedPtr & msg)
{
  rerun_bridge_->log_static_transforms(*msg);
}

void RerunWrapperNode::handle_synced_frame(
  const sensor_msgs::msg::Image::ConstSharedPtr & left_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & right_msg,
  const nav_msgs::msg::Odometry::ConstSharedPtr & ground_truth_odom_msg,
  const nav_msgs::msg::Odometry::ConstSharedPtr & stereo_vio_odom_msg,
  const stereo_vio_msgs::msg::StereoObservations::ConstSharedPtr &
    stereo_vio_observations_msg)
{
  rerun_bridge_->log_synced_frame(
    *left_msg,
    *right_msg,
    *ground_truth_odom_msg,
    *stereo_vio_odom_msg,
    *stereo_vio_observations_msg);
}

}  // namespace rerun_wrapper

RCLCPP_COMPONENTS_REGISTER_NODE(rerun_wrapper::RerunWrapperNode)
