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

#include "stereo_vio/odometry_error_node.hpp"

#include "rclcpp_components/register_node_macro.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <cstddef>
#include <cstdint>
#include <cmath>
#include <filesystem>
#include <functional>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>

namespace
{

constexpr char kGroundTruthOdomTopic[] = "ground_truth/odom";
constexpr char kStereoVioOdomTopic[] = "stereo_vio/odom";
constexpr char kErrorTopic[] = "errors";
constexpr char kDefaultErrorReportPath[] = "/tmp/stereo_vio_odometry_error.txt";
constexpr int64_t kDefaultSyncQueueSize = 500;
constexpr double kDefaultSyncTimeSeconds = 0.05;
constexpr double kRadiansToDegrees = 57.2957795130823208768;
constexpr double kNanosecondsToSeconds = 1.0e-9;
constexpr int kErrorReportPrecision = 4;

[[nodiscard]] rclcpp::QoS make_error_node_qos(const std::size_t queue_depth)
{
  rclcpp::QoS qos{queue_depth};
  qos.reliable();
  qos.durability_volatile();
  return qos;
}

[[nodiscard]] Eigen::Isometry3d odometry_pose_to_isometry(
  const nav_msgs::msg::Odometry & odometry)
{
  const geometry_msgs::msg::Point & position = odometry.pose.pose.position;
  const geometry_msgs::msg::Quaternion & orientation = odometry.pose.pose.orientation;

  Eigen::Quaterniond rotation(
    orientation.w,
    orientation.x,
    orientation.y,
    orientation.z);
  if (rotation.norm() <= 0.0) {
    throw std::invalid_argument("Odometry pose has a zero-norm quaternion");
  }
  rotation.normalize();

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = rotation.toRotationMatrix();
  pose.translation() = Eigen::Vector3d(position.x, position.y, position.z);
  return pose;
}

[[nodiscard]] double rotation_angle_degrees(const Eigen::Matrix3d & rotation)
{
  const Eigen::AngleAxisd angle_axis(rotation);
  return std::abs(angle_axis.angle()) * kRadiansToDegrees;
}

[[nodiscard]] double translation_distance(
  const Eigen::Isometry3d & first,
  const Eigen::Isometry3d & second)
{
  return (first.translation() - second.translation()).norm();
}

[[nodiscard]] int64_t stamp_to_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + static_cast<int64_t>(stamp.nanosec);
}

}  // namespace

namespace stereo_vio
{

OdometryErrorNode::OdometryErrorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("odometry_error_node", options),
  sync_queue_size_(this->declare_sync_queue_size()),
  sync_time_seconds_(this->declare_sync_time_seconds()),
  error_report_path_(this->declare_error_report_path()),
  sample_count_(0U),
  rpe_sample_count_(0U),
  ate_squared_sum_(0.0),
  are_squared_sum_(0.0),
  rpe_translation_squared_sum_(0.0),
  rpe_rotation_squared_sum_(0.0),
  gt_distance_m_(0.0),
  vio_distance_m_(0.0)
{
  this->sync_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sync_options =
    this->make_subscription_options(this->sync_callback_group_);
  const rclcpp::QoS qos =
    make_error_node_qos(static_cast<std::size_t>(this->sync_queue_size_));

  this->ground_truth_odom_sub_.subscribe(
    this,
    kGroundTruthOdomTopic,
    qos.get_rmw_qos_profile(),
    sync_options);
  this->stereo_vio_odom_sub_.subscribe(
    this,
    kStereoVioOdomTopic,
    qos.get_rmw_qos_profile(),
    sync_options);

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry,
    nav_msgs::msg::Odometry>;
  this->odometry_sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(static_cast<uint32_t>(this->sync_queue_size_)),
    this->ground_truth_odom_sub_,
    this->stereo_vio_odom_sub_);
  this->odometry_sync_->setMaxIntervalDuration(
    rclcpp::Duration::from_seconds(this->sync_time_seconds_));
  this->odometry_sync_->registerCallback(
    std::bind(
      &OdometryErrorNode::handle_synced_odometry,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  this->error_pub_ =
    this->create_publisher<stereo_vio_msgs::msg::OdometryError>(kErrorTopic, qos);

  this->initialize_error_report_file();
}

int64_t OdometryErrorNode::declare_sync_queue_size()
{
  const int64_t sync_queue_size =
    this->declare_parameter<int64_t>("sync_queue_size", kDefaultSyncQueueSize);
  if (sync_queue_size < 2) {
    throw std::invalid_argument("Parameter 'sync_queue_size' must be at least 2");
  }
  return sync_queue_size;
}

double OdometryErrorNode::declare_sync_time_seconds()
{
  const double sync_time_seconds =
    this->declare_parameter<double>("sync_time_seconds", kDefaultSyncTimeSeconds);
  if (sync_time_seconds <= 0.0) {
    throw std::invalid_argument("Parameter 'sync_time_seconds' must be positive");
  }
  return sync_time_seconds;
}

std::string OdometryErrorNode::declare_error_report_path()
{
  return this->declare_parameter<std::string>("error_report_path", kDefaultErrorReportPath);
}

rclcpp::SubscriptionOptions OdometryErrorNode::make_subscription_options(
  const rclcpp::CallbackGroup::SharedPtr & callback_group) const
{
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group;
  return options;
}

void OdometryErrorNode::initialize_error_report_file() const
{
  if (this->error_report_path_.empty()) {
    return;
  }

  try {
    const std::filesystem::path report_path{this->error_report_path_};
    const std::filesystem::path parent_path = report_path.parent_path();
    if (!parent_path.empty()) {
      std::filesystem::create_directories(parent_path);
    }

    std::ofstream report_file(report_path, std::ios::out | std::ios::trunc);
    if (!report_file.is_open()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to open odometry error report file for overwrite: %s",
        this->error_report_path_.c_str());
      return;
    }

    report_file << "Stereo VIO odometry error summary\n";
    report_file << "Waiting for synchronized odometry samples.\n";
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      this->get_logger(),
      "Failed to initialize odometry error report file '%s': %s",
      this->error_report_path_.c_str(),
      error.what());
  }
}

void OdometryErrorNode::write_error_report(
  const stereo_vio_msgs::msg::OdometryError & error) const
{
  if (this->error_report_path_.empty()) {
    return;
  }

  try {
    const std::filesystem::path report_path{this->error_report_path_};
    std::ofstream report_file(report_path, std::ios::out | std::ios::trunc);
    if (!report_file.is_open()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to open odometry error report file for overwrite: %s",
        this->error_report_path_.c_str());
      return;
    }

    const double distance_error_m = std::abs(error.vio_distance_m - error.gt_distance_m);
    const int64_t end_timestamp_ns = stamp_to_ns(error.header.stamp);
    int64_t start_timestamp_ns = end_timestamp_ns;
    if (this->start_timestamp_ns_.has_value()) {
      start_timestamp_ns = this->start_timestamp_ns_.value();
    }
    const double duration_s =
      static_cast<double>(end_timestamp_ns - start_timestamp_ns) * kNanosecondsToSeconds;

    report_file << std::fixed << std::setprecision(kErrorReportPrecision);
    report_file << "Stereo VIO odometry error summary\n";
    report_file << "Duration: " << duration_s << " s";
    report_file << " | Samples: " << error.sample_count;
    report_file << " | RPE samples: " << error.rpe_sample_count << "\n\n";

    report_file << "Absolute pose error\n";
    report_file << "  ATE [m]:       final " << error.ate_m;
    report_file << " | RMSE " << error.ate_rmse_m << "\n";
    report_file << "  ARE [deg]:     final " << error.are_deg;
    report_file << " | RMSE " << error.are_rmse_deg << "\n\n";

    report_file << "Relative pose error\n";
    report_file << "  Translation [m]:   final " << error.rpe_translation_m;
    report_file << " | RMSE " << error.rpe_translation_rmse_m << "\n";
    report_file << "  Rotation [deg]:    final " << error.rpe_rotation_deg;
    report_file << " | RMSE " << error.rpe_rotation_rmse_deg << "\n\n";

    report_file << "Distance travelled\n";
    report_file << "  Ground truth [m]:  " << error.gt_distance_m << "\n";
    report_file << "  Stereo VIO [m]:    " << error.vio_distance_m << "\n";
    report_file << "  Difference [m]:    " << distance_error_m << "\n";
  } catch (const std::exception & write_error) {
    RCLCPP_WARN(
      this->get_logger(),
      "Failed to write odometry error report file '%s': %s",
      this->error_report_path_.c_str(),
      write_error.what());
  }
}

void OdometryErrorNode::handle_synced_odometry(
  const nav_msgs::msg::Odometry::ConstSharedPtr & ground_truth_msg,
  const nav_msgs::msg::Odometry::ConstSharedPtr & stereo_vio_msg)
{
  try {
    const int64_t ground_truth_timestamp_ns = stamp_to_ns(ground_truth_msg->header.stamp);
    if (!this->start_timestamp_ns_.has_value()) {
      this->start_timestamp_ns_ = ground_truth_timestamp_ns;
    }

    const Eigen::Isometry3d gt_pose = odometry_pose_to_isometry(*ground_truth_msg);
    const Eigen::Isometry3d vio_pose = odometry_pose_to_isometry(*stereo_vio_msg);

    if (!this->alignment_gt_from_vio_.has_value()) {
      this->alignment_gt_from_vio_ = gt_pose * vio_pose.inverse();
    }

    const Eigen::Isometry3d aligned_vio_pose =
      this->alignment_gt_from_vio_.value() * vio_pose;
    const Eigen::Isometry3d absolute_error = gt_pose.inverse() * aligned_vio_pose;
    const double ate_m = absolute_error.translation().norm();
    const double are_deg = rotation_angle_degrees(absolute_error.linear());

    ++this->sample_count_;
    this->ate_squared_sum_ += ate_m * ate_m;
    this->are_squared_sum_ += are_deg * are_deg;

    bool rpe_valid = false;
    double rpe_translation_m = 0.0;
    double rpe_rotation_deg = 0.0;

    if (this->previous_gt_pose_.has_value() &&
      this->previous_aligned_vio_pose_.has_value())
    {
      this->gt_distance_m_ += translation_distance(this->previous_gt_pose_.value(), gt_pose);
      this->vio_distance_m_ +=
        translation_distance(this->previous_aligned_vio_pose_.value(), aligned_vio_pose);

      const Eigen::Isometry3d gt_delta =
        this->previous_gt_pose_.value().inverse() * gt_pose;
      const Eigen::Isometry3d vio_delta =
        this->previous_aligned_vio_pose_.value().inverse() * aligned_vio_pose;
      const Eigen::Isometry3d relative_error = gt_delta.inverse() * vio_delta;
      rpe_translation_m = relative_error.translation().norm();
      rpe_rotation_deg = rotation_angle_degrees(relative_error.linear());
      rpe_valid = true;

      ++this->rpe_sample_count_;
      this->rpe_translation_squared_sum_ += rpe_translation_m * rpe_translation_m;
      this->rpe_rotation_squared_sum_ += rpe_rotation_deg * rpe_rotation_deg;
    }

    this->previous_gt_pose_ = gt_pose;
    this->previous_aligned_vio_pose_ = aligned_vio_pose;

    stereo_vio_msgs::msg::OdometryError error_msg;
    error_msg.header.stamp = ground_truth_msg->header.stamp;
    error_msg.header.frame_id = ground_truth_msg->header.frame_id;
    error_msg.aligned = this->alignment_gt_from_vio_.has_value();
    error_msg.rpe_valid = rpe_valid;
    error_msg.sample_count = this->sample_count_;
    error_msg.rpe_sample_count = this->rpe_sample_count_;
    error_msg.ate_m = ate_m;
    error_msg.are_deg = are_deg;
    error_msg.ate_rmse_m =
      std::sqrt(this->ate_squared_sum_ / static_cast<double>(this->sample_count_));
    error_msg.are_rmse_deg =
      std::sqrt(this->are_squared_sum_ / static_cast<double>(this->sample_count_));
    error_msg.rpe_translation_m = rpe_translation_m;
    error_msg.rpe_rotation_deg = rpe_rotation_deg;
    if (this->rpe_sample_count_ > 0U) {
      error_msg.rpe_translation_rmse_m =
        std::sqrt(
          this->rpe_translation_squared_sum_ /
          static_cast<double>(this->rpe_sample_count_));
      error_msg.rpe_rotation_rmse_deg =
        std::sqrt(
          this->rpe_rotation_squared_sum_ /
          static_cast<double>(this->rpe_sample_count_));
    }
    error_msg.gt_distance_m = this->gt_distance_m_;
    error_msg.vio_distance_m = this->vio_distance_m_;
    this->error_pub_->publish(error_msg);
    this->write_error_report(error_msg);
  } catch (const std::exception & error) {
    RCLCPP_WARN(this->get_logger(), "Failed to compute odometry error: %s", error.what());
  }
}

}  // namespace stereo_vio

RCLCPP_COMPONENTS_REGISTER_NODE(stereo_vio::OdometryErrorNode)
