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

#include "stereo_vio/vio_backend.hpp"
#include "stereo_vio/vio_frontend.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "stereo_vio_msgs/msg/stereo_observations.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace stereo_vio
{

class VioNode final : public rclcpp::Node
{
public:
  explicit VioNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~VioNode() override = default;

  VioNode(const VioNode &) = delete;
  VioNode & operator=(const VioNode &) = delete;
  VioNode(VioNode &&) = delete;
  VioNode & operator=(VioNode &&) = delete;

private:
  struct CameraIntrinsics
  {
    std::string frame_id;
    double fx;
    double fy;
    double cx;
    double cy;
    double projection_tx;
  };

  [[nodiscard]] int64_t declare_sync_queue_size();
  [[nodiscard]] double declare_sync_time_seconds();
  [[nodiscard]] VioFrontend::Options declare_frontend_options();
  [[nodiscard]] VioBackendOptions declare_backend_options();
  [[nodiscard]] bool is_camera_info_valid(
    const sensor_msgs::msg::CameraInfo & camera_info) const;
  [[nodiscard]] CameraIntrinsics make_intrinsics(
    const sensor_msgs::msg::CameraInfo & camera_info) const;
  [[nodiscard]] rclcpp::SubscriptionOptions make_subscription_options(
    const rclcpp::CallbackGroup::SharedPtr & callback_group) const;
  [[nodiscard]] std::optional<Eigen::Isometry3d> lookup_static_transform_locked(
    const std::string & target_frame,
    const std::string & source_frame) const;
  [[nodiscard]] std::optional<VioBackendFrameTransforms> make_backend_frame_transforms_locked(
    const std::string & left_camera_frame) const;
  [[nodiscard]] std::vector<VioImuSample> copy_latest_imu_samples_locked() const;
  [[nodiscard]] std::vector<VioImuSample> take_imu_samples_until_locked(
    const builtin_interfaces::msg::Time & stamp);
  [[nodiscard]] std::optional<VioImuSample> make_imu_sample_locked(
    const sensor_msgs::msg::Imu & imu_msg) const;

  void setup_stereo_sync(const rclcpp::QoS & qos);
  void handle_left_camera_info(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg);
  void handle_right_camera_info(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg);
  void handle_imu(const sensor_msgs::msg::Imu::ConstSharedPtr & imu_msg);
  void handle_static_tf(const tf2_msgs::msg::TFMessage::ConstSharedPtr & tf_msg);
  void handle_stereo_images(
    const sensor_msgs::msg::Image::ConstSharedPtr & left_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & right_msg);
  void publish_backend_result(
    const VioBackendResult & backend_result,
    const builtin_interfaces::msg::Time & stamp);
  void publish_stereo_observations(
    const VioFrontendResult & frontend_result,
    const builtin_interfaces::msg::Time & stamp,
    const std::string & frame_id);

  int64_t sync_queue_size_;
  double sync_time_seconds_;
  std::string base_frame_id_;
  std::string imu_frame_id_;
  std::string odom_frame_id_;
  std::optional<CameraIntrinsics> left_intrinsics_;
  std::optional<CameraIntrinsics> right_intrinsics_;
  std::deque<sensor_msgs::msg::Imu> imu_buffer_;
  std::map<std::string, geometry_msgs::msg::TransformStamped> static_transforms_;
  bool backend_initialized_;
  VioFrontend frontend_;
  VioBackend backend_;
  mutable std::mutex state_mutex_;
  mutable std::mutex frontend_mutex_;
  mutable std::mutex backend_mutex_;
  rclcpp::CallbackGroup::SharedPtr stereo_callback_group_;
  rclcpp::CallbackGroup::SharedPtr camera_info_callback_group_;
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::CallbackGroup::SharedPtr tf_static_callback_group_;
  message_filters::Subscriber<sensor_msgs::msg::Image> left_rect_image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> right_rect_image_sub_;
  std::shared_ptr<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image>>>
    stereo_sync_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<stereo_vio_msgs::msg::StereoObservations>::SharedPtr observations_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster_;
};

}  // namespace stereo_vio
