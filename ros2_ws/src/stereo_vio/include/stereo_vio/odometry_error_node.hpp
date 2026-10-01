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

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "stereo_vio_msgs/msg/odometry_error.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include <Eigen/Geometry>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace stereo_vio
{

class OdometryErrorNode final : public rclcpp::Node
{
public:
  explicit OdometryErrorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~OdometryErrorNode() override = default;

  OdometryErrorNode(const OdometryErrorNode &) = delete;
  OdometryErrorNode & operator=(const OdometryErrorNode &) = delete;
  OdometryErrorNode(OdometryErrorNode &&) = delete;
  OdometryErrorNode & operator=(OdometryErrorNode &&) = delete;

private:
  [[nodiscard]] int64_t declare_sync_queue_size();
  [[nodiscard]] double declare_sync_time_seconds();
  [[nodiscard]] std::string declare_error_report_path();
  [[nodiscard]] rclcpp::SubscriptionOptions make_subscription_options(
    const rclcpp::CallbackGroup::SharedPtr & callback_group) const;

  void initialize_error_report_file() const;
  void write_error_report(const stereo_vio_msgs::msg::OdometryError & error) const;
  void handle_synced_odometry(
    const nav_msgs::msg::Odometry::ConstSharedPtr & ground_truth_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr & stereo_vio_msg);

  int64_t sync_queue_size_;
  double sync_time_seconds_;
  std::string error_report_path_;
  rclcpp::CallbackGroup::SharedPtr sync_callback_group_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> ground_truth_odom_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> stereo_vio_odom_sub_;
  std::shared_ptr<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<
        nav_msgs::msg::Odometry,
        nav_msgs::msg::Odometry>>>
    odometry_sync_;
  rclcpp::Publisher<stereo_vio_msgs::msg::OdometryError>::SharedPtr error_pub_;

  std::optional<Eigen::Isometry3d> alignment_gt_from_vio_;
  std::optional<Eigen::Isometry3d> previous_gt_pose_;
  std::optional<Eigen::Isometry3d> previous_aligned_vio_pose_;
  std::optional<int64_t> start_timestamp_ns_;
  uint64_t sample_count_;
  uint64_t rpe_sample_count_;
  double ate_squared_sum_;
  double are_squared_sum_;
  double rpe_translation_squared_sum_;
  double rpe_rotation_squared_sum_;
  double gt_distance_m_;
  double vio_distance_m_;
};

}  // namespace stereo_vio
