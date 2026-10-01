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

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "stereo_vio_msgs/msg/odometry_error.hpp"
#include "stereo_vio_msgs/msg/stereo_observations.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include <memory>
#include <optional>

namespace rerun_wrapper
{

class RerunBridge;

class RerunWrapperNode final : public rclcpp::Node
{
public:
  explicit RerunWrapperNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RerunWrapperNode() override;

  RerunWrapperNode(const RerunWrapperNode &) = delete;
  RerunWrapperNode & operator=(const RerunWrapperNode &) = delete;
  RerunWrapperNode(RerunWrapperNode &&) = delete;
  RerunWrapperNode & operator=(RerunWrapperNode &&) = delete;

private:
  void handle_left_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);
  void handle_right_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);
  void handle_imu(const sensor_msgs::msg::Imu::ConstSharedPtr & msg);
  void handle_odometry_error(const stereo_vio_msgs::msg::OdometryError::ConstSharedPtr & msg);
  void handle_tf_static(const tf2_msgs::msg::TFMessage::ConstSharedPtr & msg);
  void handle_synced_frame(
    const sensor_msgs::msg::Image::ConstSharedPtr & left_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & right_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr & ground_truth_odom_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr & stereo_vio_odom_msg,
    const stereo_vio_msgs::msg::StereoObservations::ConstSharedPtr &
      stereo_vio_observations_msg);

  rclcpp::CallbackGroup::SharedPtr camera_info_callback_group_;
  rclcpp::CallbackGroup::SharedPtr sync_callback_group_;
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::CallbackGroup::SharedPtr odometry_error_callback_group_;
  rclcpp::CallbackGroup::SharedPtr tf_static_callback_group_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<stereo_vio_msgs::msg::OdometryError>::SharedPtr odometry_error_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;

  message_filters::Subscriber<sensor_msgs::msg::Image> left_image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> right_image_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> ground_truth_odom_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> stereo_vio_odom_sub_;
  message_filters::Subscriber<stereo_vio_msgs::msg::StereoObservations>
    stereo_vio_observations_sub_;
  std::shared_ptr<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image,
        nav_msgs::msg::Odometry,
        nav_msgs::msg::Odometry,
        stereo_vio_msgs::msg::StereoObservations>>>
    synced_frame_sync_;

  std::optional<sensor_msgs::msg::CameraInfo> left_camera_info_;
  std::optional<sensor_msgs::msg::CameraInfo> right_camera_info_;

  std::unique_ptr<RerunBridge> rerun_bridge_;
};

}  // namespace rerun_wrapper
