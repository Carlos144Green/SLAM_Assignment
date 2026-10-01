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

#include "stereo_vio/vio_node.hpp"

#include "rclcpp_components/register_node_macro.hpp"

#include <Eigen/Geometry>

#include <geometry_msgs/msg/point32.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{

constexpr char kLeftRectImageTopic[] = "left/image_rect";
constexpr char kRightRectImageTopic[] = "right/image_rect";
constexpr char kLeftCameraInfoTopic[] = "left/camera_info";
constexpr char kRightCameraInfoTopic[] = "right/camera_info";
constexpr char kImuTopic[] = "imu";
constexpr char kTfStaticTopic[] = "tf_static";
constexpr char kOdomTopic[] = "odom";
constexpr char kStereoObservationsTopic[] = "observations";

constexpr int64_t kDefaultSyncQueueSize = 240;
constexpr double kDefaultSyncTimeSeconds = 0.01;
constexpr int64_t kDefaultPixelMaskBorder = 20;
constexpr std::size_t kMaxImuBufferSize = 20000U;
constexpr std::size_t kMaxInitialGravitySampleCount = 1000U;
constexpr char kDefaultBaseFrameId[] = "base_link";
constexpr char kDefaultImuFrameId[] = "oak_imu_link";
constexpr char kDefaultOdomFrameId[] = "odom";

[[nodiscard]] rclcpp::QoS make_vio_data_qos(const std::size_t queue_depth)
{
  rclcpp::QoS qos{queue_depth};
  qos.reliable();
  qos.durability_volatile();
  return qos;
}

[[nodiscard]] rclcpp::QoS make_static_tf_qos()
{
  rclcpp::QoS qos{1U};
  qos.reliable();
  qos.transient_local();
  return qos;
}

[[nodiscard]] std::string make_static_transform_key(
  const geometry_msgs::msg::TransformStamped & transform)
{
  return transform.header.frame_id + "->" + transform.child_frame_id;
}

[[nodiscard]] double stamp_to_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

[[nodiscard]] bool is_stamp_less_equal(
  const builtin_interfaces::msg::Time & left,
  const builtin_interfaces::msg::Time & right)
{
  if (left.sec < right.sec) {
    return true;
  }
  if (left.sec > right.sec) {
    return false;
  }
  return left.nanosec <= right.nanosec;
}

[[nodiscard]] Eigen::Isometry3d make_isometry3d(
  const geometry_msgs::msg::TransformStamped & transform_msg)
{
  Eigen::Quaterniond rotation(
    transform_msg.transform.rotation.w,
    transform_msg.transform.rotation.x,
    transform_msg.transform.rotation.y,
    transform_msg.transform.rotation.z);
  if (rotation.norm() <= 0.0) {
    throw std::invalid_argument("Static transform has a zero-norm quaternion");
  }
  rotation.normalize();

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = rotation.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(
    transform_msg.transform.translation.x,
    transform_msg.transform.translation.y,
    transform_msg.transform.translation.z);
  return transform;
}

[[nodiscard]] geometry_msgs::msg::Quaternion make_quaternion_msg(
  const Eigen::Matrix3d & rotation_matrix)
{
  Eigen::Quaterniond rotation(rotation_matrix);
  rotation.normalize();

  geometry_msgs::msg::Quaternion quaternion_msg;
  quaternion_msg.x = rotation.x();
  quaternion_msg.y = rotation.y();
  quaternion_msg.z = rotation.z();
  quaternion_msg.w = rotation.w();
  return quaternion_msg;
}

void fill_transform_msg(
  geometry_msgs::msg::Transform & transform_msg,
  const Eigen::Isometry3d & transform)
{
  transform_msg.translation.x = transform.translation().x();
  transform_msg.translation.y = transform.translation().y();
  transform_msg.translation.z = transform.translation().z();
  transform_msg.rotation = make_quaternion_msg(transform.linear());
}

void fill_pose_msg(
  geometry_msgs::msg::Pose & pose_msg,
  const Eigen::Isometry3d & transform)
{
  pose_msg.position.x = transform.translation().x();
  pose_msg.position.y = transform.translation().y();
  pose_msg.position.z = transform.translation().z();
  pose_msg.orientation = make_quaternion_msg(transform.linear());
}

[[nodiscard]] cv::Mat make_cv_image(const sensor_msgs::msg::Image & image_msg)
{
  if (image_msg.width == 0U || image_msg.height == 0U) {
    throw std::invalid_argument("Image message dimensions cannot be zero");
  }

  int type = CV_8UC1;
  std::size_t bytes_per_pixel = 1U;
  if (image_msg.encoding == "mono8" || image_msg.encoding == "8UC1") {
    type = CV_8UC1;
    bytes_per_pixel = 1U;
  } else if (image_msg.encoding == "bgr8" || image_msg.encoding == "rgb8" ||
             image_msg.encoding == "8UC3")
  {
    type = CV_8UC3;
    bytes_per_pixel = 3U;
  } else if (image_msg.encoding == "bgra8" || image_msg.encoding == "rgba8" ||
             image_msg.encoding == "8UC4")
  {
    type = CV_8UC4;
    bytes_per_pixel = 4U;
  } else {
    throw std::invalid_argument("Unsupported image encoding: " + image_msg.encoding);
  }

  const std::size_t minimum_step = static_cast<std::size_t>(image_msg.width) * bytes_per_pixel;
  if (image_msg.step < minimum_step) {
    throw std::invalid_argument("Image message step is smaller than image width");
  }

  const std::size_t required_bytes =
    static_cast<std::size_t>(image_msg.step) *
    static_cast<std::size_t>(image_msg.height - 1U) + minimum_step;
  if (image_msg.data.size() < required_bytes) {
    throw std::invalid_argument("Image message data is smaller than expected");
  }

  cv::Mat raw_image(
    static_cast<int>(image_msg.height),
    static_cast<int>(image_msg.width),
    type,
    const_cast<unsigned char *>(image_msg.data.data()),
    image_msg.step);

  cv::Mat converted_image;
  if (image_msg.encoding == "rgb8") {
    cv::cvtColor(raw_image, converted_image, cv::COLOR_RGB2BGR);
    return converted_image;
  }
  if (image_msg.encoding == "rgba8") {
    cv::cvtColor(raw_image, converted_image, cv::COLOR_RGBA2BGR);
    return converted_image;
  }
  if (image_msg.encoding == "bgra8") {
    cv::cvtColor(raw_image, converted_image, cv::COLOR_BGRA2BGR);
    return converted_image;
  }
  return raw_image.clone();
}

[[nodiscard]] bool is_observation_publishable(
  const stereo_vio::VioFeatureObservation & observation)
{
  return observation.triangulated &&
         std::isfinite(observation.left_pixel.x) &&
         std::isfinite(observation.left_pixel.y) &&
         std::isfinite(observation.right_pixel.x) &&
         std::isfinite(observation.right_pixel.y) &&
         std::isfinite(observation.landmark_left.x) &&
         std::isfinite(observation.landmark_left.y) &&
         std::isfinite(observation.landmark_left.z) &&
         observation.landmark_left.z > 0.0;
}

[[nodiscard]] geometry_msgs::msg::Point32 point32_from_cv_point(
  const cv::Point3d & point)
{
  geometry_msgs::msg::Point32 point_msg;
  point_msg.x = static_cast<float>(point.x);
  point_msg.y = static_cast<float>(point.y);
  point_msg.z = static_cast<float>(point.z);
  return point_msg;
}

}  // namespace

namespace stereo_vio
{

VioNode::VioNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("vio_node", options),
  sync_queue_size_(this->declare_sync_queue_size()),
  sync_time_seconds_(this->declare_sync_time_seconds()),
  base_frame_id_(this->declare_parameter<std::string>("base_frame_id", kDefaultBaseFrameId)),
  imu_frame_id_(this->declare_parameter<std::string>("imu_frame_id", kDefaultImuFrameId)),
  odom_frame_id_(this->declare_parameter<std::string>("odom_frame_id", kDefaultOdomFrameId)),
  backend_initialized_(false),
  frontend_(this->declare_frontend_options()),
  backend_(this->declare_backend_options())
{
  const rclcpp::QoS qos = make_vio_data_qos(static_cast<std::size_t>(this->sync_queue_size_));
  this->stereo_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->camera_info_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->imu_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  this->tf_static_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  const rclcpp::SubscriptionOptions camera_info_subscription_options =
    this->make_subscription_options(this->camera_info_callback_group_);
  const rclcpp::SubscriptionOptions imu_subscription_options =
    this->make_subscription_options(this->imu_callback_group_);
  const rclcpp::SubscriptionOptions tf_static_subscription_options =
    this->make_subscription_options(this->tf_static_callback_group_);

  this->left_camera_info_sub_ =
    this->create_subscription<sensor_msgs::msg::CameraInfo>(
      kLeftCameraInfoTopic,
      qos,
      std::bind(
        &VioNode::handle_left_camera_info,
        this,
        std::placeholders::_1),
      camera_info_subscription_options);
  this->right_camera_info_sub_ =
    this->create_subscription<sensor_msgs::msg::CameraInfo>(
      kRightCameraInfoTopic,
      qos,
      std::bind(
        &VioNode::handle_right_camera_info,
        this,
        std::placeholders::_1),
      camera_info_subscription_options);
  this->imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    kImuTopic,
    qos,
    std::bind(&VioNode::handle_imu, this, std::placeholders::_1),
    imu_subscription_options);
  this->tf_static_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
    kTfStaticTopic,
    make_static_tf_qos(),
    std::bind(&VioNode::handle_static_tf, this, std::placeholders::_1),
    tf_static_subscription_options);
  this->odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(kOdomTopic, qos);
  this->observations_pub_ =
    this->create_publisher<stereo_vio_msgs::msg::StereoObservations>(
      kStereoObservationsTopic,
      qos);
  this->odom_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  this->setup_stereo_sync(qos);
}

int64_t VioNode::declare_sync_queue_size()
{
  const int64_t sync_queue_size =
    this->declare_parameter<int64_t>("sync_queue_size", kDefaultSyncQueueSize);
  if (sync_queue_size < 2) {
    throw std::invalid_argument("Parameter 'sync_queue_size' must be at least 2");
  }
  return sync_queue_size;
}

double VioNode::declare_sync_time_seconds()
{
  const double sync_time_seconds =
    this->declare_parameter<double>("sync_time_seconds", kDefaultSyncTimeSeconds);
  if (sync_time_seconds <= 0.0) {
    throw std::invalid_argument("Parameter 'sync_time_seconds' must be positive");
  }
  return sync_time_seconds;
}

VioFrontend::Options VioNode::declare_frontend_options()
{
  const int64_t pixel_mask_border =
    this->declare_parameter<int64_t>("pixel_mask_border", kDefaultPixelMaskBorder);
  if (pixel_mask_border < 0) {
    throw std::invalid_argument("Parameter 'pixel_mask_border' cannot be negative");
  }

  VioFrontend::Options options;
  options.pixel_mask_border = static_cast<int>(pixel_mask_border);
  return options;
}

VioBackendOptions VioNode::declare_backend_options()
{
  return VioBackendOptions{};
}

bool VioNode::is_camera_info_valid(
  const sensor_msgs::msg::CameraInfo & camera_info) const
{
  return camera_info.width > 0U && camera_info.height > 0U &&
         camera_info.k[0] > 0.0 && camera_info.k[4] > 0.0;
}

VioNode::CameraIntrinsics VioNode::make_intrinsics(
  const sensor_msgs::msg::CameraInfo & camera_info) const
{
  return {
    camera_info.header.frame_id,
    camera_info.k[0],
    camera_info.k[4],
    camera_info.k[2],
    camera_info.k[5],
    camera_info.p[3]};
}

rclcpp::SubscriptionOptions VioNode::make_subscription_options(
  const rclcpp::CallbackGroup::SharedPtr & callback_group) const
{
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = callback_group;
  return subscription_options;
}

std::optional<Eigen::Isometry3d> VioNode::lookup_static_transform_locked(
  const std::string & target_frame,
  const std::string & source_frame) const
{
  if (target_frame == source_frame) {
    return Eigen::Isometry3d::Identity();
  }

  std::map<std::string, std::vector<std::pair<std::string, Eigen::Isometry3d>>> graph;
  for (
    const std::pair<const std::string, geometry_msgs::msg::TransformStamped> &
    transform_entry : this->static_transforms_)
  {
    const geometry_msgs::msg::TransformStamped & transform_msg = transform_entry.second;
    const Eigen::Isometry3d parent_T_child = make_isometry3d(transform_msg);
    graph[transform_msg.header.frame_id].push_back(
      std::make_pair(transform_msg.child_frame_id, parent_T_child));
    graph[transform_msg.child_frame_id].push_back(
      std::make_pair(transform_msg.header.frame_id, parent_T_child.inverse()));
  }

  std::deque<std::pair<std::string, Eigen::Isometry3d>> queue;
  std::set<std::string> visited_frames;
  queue.push_back(std::make_pair(target_frame, Eigen::Isometry3d::Identity()));
  visited_frames.insert(target_frame);

  while (!queue.empty()) {
    const std::pair<std::string, Eigen::Isometry3d> current = queue.front();
    queue.pop_front();

    const std::map<
      std::string,
      std::vector<std::pair<std::string, Eigen::Isometry3d>>>::const_iterator edge_iterator =
      graph.find(current.first);
    if (edge_iterator == graph.end()) {
      continue;
    }

    for (const std::pair<std::string, Eigen::Isometry3d> & edge : edge_iterator->second) {
      if (visited_frames.find(edge.first) != visited_frames.end()) {
        continue;
      }

      const Eigen::Isometry3d target_T_neighbor = current.second * edge.second;
      if (edge.first == source_frame) {
        return target_T_neighbor;
      }
      queue.push_back(std::make_pair(edge.first, target_T_neighbor));
      visited_frames.insert(edge.first);
    }
  }

  return std::nullopt;
}

std::optional<VioBackendFrameTransforms> VioNode::make_backend_frame_transforms_locked(
  const std::string & left_camera_frame) const
{
  const std::optional<Eigen::Isometry3d> base_T_imu =
    this->lookup_static_transform_locked(this->base_frame_id_, this->imu_frame_id_);
  const std::optional<Eigen::Isometry3d> base_T_left_camera =
    this->lookup_static_transform_locked(this->base_frame_id_, left_camera_frame);

  if (!base_T_imu.has_value() || !base_T_left_camera.has_value()) {
    return std::nullopt;
  }

  VioBackendFrameTransforms frame_transforms;
  frame_transforms.imu_T_base = base_T_imu.value().inverse();
  frame_transforms.imu_T_left_camera =
    frame_transforms.imu_T_base * base_T_left_camera.value();
  return frame_transforms;
}

std::vector<VioImuSample> VioNode::copy_latest_imu_samples_locked() const
{
  std::vector<VioImuSample> imu_samples;
  imu_samples.reserve(std::min(this->imu_buffer_.size(), kMaxInitialGravitySampleCount));

  std::size_t skipped_sample_count =
    this->imu_buffer_.size() > kMaxInitialGravitySampleCount ?
    this->imu_buffer_.size() - kMaxInitialGravitySampleCount :
    0U;
  for (const sensor_msgs::msg::Imu & imu_msg : this->imu_buffer_) {
    if (skipped_sample_count > 0U) {
      --skipped_sample_count;
      continue;
    }

    const std::optional<VioImuSample> imu_sample = this->make_imu_sample_locked(imu_msg);
    if (imu_sample.has_value()) {
      imu_samples.push_back(imu_sample.value());
    }
  }
  return imu_samples;
}

std::vector<VioImuSample> VioNode::take_imu_samples_until_locked(
  const builtin_interfaces::msg::Time & stamp)
{
  std::vector<VioImuSample> imu_samples;
  while (
    !this->imu_buffer_.empty() &&
    is_stamp_less_equal(this->imu_buffer_.front().header.stamp, stamp))
  {
    const std::optional<VioImuSample> imu_sample =
      this->make_imu_sample_locked(this->imu_buffer_.front());
    if (imu_sample.has_value()) {
      imu_samples.push_back(imu_sample.value());
    }
    this->imu_buffer_.pop_front();
  }
  return imu_samples;
}

std::optional<VioImuSample> VioNode::make_imu_sample_locked(
  const sensor_msgs::msg::Imu & imu_msg) const
{
  const std::string imu_msg_frame_id =
    imu_msg.header.frame_id.empty() ? this->imu_frame_id_ : imu_msg.header.frame_id;
  const std::optional<Eigen::Isometry3d> configured_imu_T_msg_imu =
    this->lookup_static_transform_locked(this->imu_frame_id_, imu_msg_frame_id);
  if (!configured_imu_T_msg_imu.has_value()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Dropping IMU sample while waiting for static transform from '%s' to '%s'",
      this->imu_frame_id_.c_str(),
      imu_msg_frame_id.c_str());
    return std::nullopt;
  }

  const Eigen::Matrix3d configured_imu_R_msg_imu =
    configured_imu_T_msg_imu.value().linear();

  VioImuSample imu_sample;
  imu_sample.stamp_seconds = stamp_to_seconds(imu_msg.header.stamp);
  imu_sample.linear_acceleration = configured_imu_R_msg_imu * Eigen::Vector3d(
    imu_msg.linear_acceleration.x,
    imu_msg.linear_acceleration.y,
    imu_msg.linear_acceleration.z);
  imu_sample.angular_velocity = configured_imu_R_msg_imu * Eigen::Vector3d(
    imu_msg.angular_velocity.x,
    imu_msg.angular_velocity.y,
    imu_msg.angular_velocity.z);
  return imu_sample;
}

void VioNode::setup_stereo_sync(const rclcpp::QoS & qos)
{
  const rclcpp::SubscriptionOptions stereo_subscription_options =
    this->make_subscription_options(this->stereo_callback_group_);

  this->left_rect_image_sub_.subscribe(
    this,
    kLeftRectImageTopic,
    qos.get_rmw_qos_profile(),
    stereo_subscription_options);
  this->right_rect_image_sub_.subscribe(
    this,
    kRightRectImageTopic,
    qos.get_rmw_qos_profile(),
    stereo_subscription_options);

  message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::Image>
    sync_policy{static_cast<uint32_t>(this->sync_queue_size_)};
  sync_policy.setMaxIntervalDuration(rclcpp::Duration::from_seconds(this->sync_time_seconds_));

  this->stereo_sync_ = std::make_shared<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image>>>(
    static_cast<const message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image,
      sensor_msgs::msg::Image> &>(sync_policy),
    this->left_rect_image_sub_,
    this->right_rect_image_sub_);
  this->stereo_sync_->registerCallback(
    std::bind(
      &VioNode::handle_stereo_images,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
}

void VioNode::handle_left_camera_info(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
{
  if (!this->is_camera_info_valid(*camera_info_msg)) {
    RCLCPP_ERROR(this->get_logger(), "Ignoring invalid left CameraInfo");
    return;
  }

  const CameraIntrinsics intrinsics = this->make_intrinsics(*camera_info_msg);
  {
    std::lock_guard<std::mutex> lock(this->state_mutex_);
    if (this->left_intrinsics_.has_value()) {
      return;
    }
    this->left_intrinsics_ = intrinsics;
  }

  this->left_camera_info_sub_.reset();
}

void VioNode::handle_right_camera_info(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
{
  if (!this->is_camera_info_valid(*camera_info_msg)) {
    RCLCPP_ERROR(this->get_logger(), "Ignoring invalid right CameraInfo");
    return;
  }

  const CameraIntrinsics intrinsics = this->make_intrinsics(*camera_info_msg);
  {
    std::lock_guard<std::mutex> lock(this->state_mutex_);
    if (this->right_intrinsics_.has_value()) {
      return;
    }
    this->right_intrinsics_ = intrinsics;
  }

  this->right_camera_info_sub_.reset();
}

void VioNode::handle_imu(const sensor_msgs::msg::Imu::ConstSharedPtr & imu_msg)
{
  std::lock_guard<std::mutex> lock(this->state_mutex_);
  this->imu_buffer_.push_back(*imu_msg);
  while (this->imu_buffer_.size() > kMaxImuBufferSize) {
    this->imu_buffer_.pop_front();
  }
}

void VioNode::handle_static_tf(const tf2_msgs::msg::TFMessage::ConstSharedPtr & tf_msg)
{
  std::lock_guard<std::mutex> lock(this->state_mutex_);
  for (const geometry_msgs::msg::TransformStamped & transform : tf_msg->transforms) {
    this->static_transforms_[make_static_transform_key(transform)] = transform;
  }
}

void VioNode::handle_stereo_images(
  const sensor_msgs::msg::Image::ConstSharedPtr & left_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & right_msg)
{
  bool waiting_for_camera_info = false;
  bool waiting_for_static_tf = false;
  CameraIntrinsics left_intrinsics{};
  CameraIntrinsics right_intrinsics{};
  VioBackendFrameTransforms backend_frame_transforms;
  std::vector<VioImuSample> imu_samples;
  {
    std::lock_guard<std::mutex> lock(this->state_mutex_);
    if (!this->left_intrinsics_.has_value() || !this->right_intrinsics_.has_value()) {
      waiting_for_camera_info = true;
    } else {
      left_intrinsics = this->left_intrinsics_.value();
      right_intrinsics = this->right_intrinsics_.value();
      const std::optional<VioBackendFrameTransforms> frame_transforms =
        this->make_backend_frame_transforms_locked(left_intrinsics.frame_id);
      if (!frame_transforms.has_value()) {
        waiting_for_static_tf = true;
      } else {
        backend_frame_transforms = frame_transforms.value();
        if (this->backend_initialized_) {
          imu_samples = this->take_imu_samples_until_locked(left_msg->header.stamp);
        } else {
          imu_samples = this->copy_latest_imu_samples_locked();
        }
      }
    }
  }

  if (waiting_for_camera_info) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Waiting for both rectified CameraInfo messages before VIO processing");
    return;
  }

  if (waiting_for_static_tf) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Waiting for static transforms from '%s' to '%s' and '%s'",
      this->base_frame_id_.c_str(),
      this->imu_frame_id_.c_str(),
      left_intrinsics.frame_id.c_str());
    return;
  }

  const double baseline = -right_intrinsics.projection_tx / right_intrinsics.fx;
  if (baseline <= 0.0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Waiting for a positive rectified stereo baseline before VIO processing");
    return;
  }

  try {
    const cv::Mat left_image = make_cv_image(*left_msg);
    const cv::Mat right_image = make_cv_image(*right_msg);
    const VioCameraModel camera_model{
      left_intrinsics.fx,
      left_intrinsics.fy,
      left_intrinsics.cx,
      left_intrinsics.cy,
      baseline};

    VioFrontendResult frontend_result;
    {
      std::lock_guard<std::mutex> lock(this->frontend_mutex_);
      frontend_result = this->frontend_.process_stereo(left_image, right_image, camera_model);
    }
    this->publish_stereo_observations(
      frontend_result,
      left_msg->header.stamp,
      left_intrinsics.frame_id);

    VioBackendResult backend_result;
    {
      std::lock_guard<std::mutex> lock(this->backend_mutex_);
      backend_result = this->backend_.process_frame(
        stamp_to_seconds(left_msg->header.stamp),
        frontend_result,
        imu_samples,
        backend_frame_transforms);
    }

    if (backend_result.valid) {
      {
        std::lock_guard<std::mutex> lock(this->state_mutex_);
        this->backend_initialized_ = true;
      }
      this->publish_backend_result(backend_result, left_msg->header.stamp);
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "VIO backend has no valid odometry yet: %s",
        backend_result.status_message.c_str());
    }
  } catch (const std::exception & exception) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Skipping VIO frame: %s",
      exception.what());
  }
}

void VioNode::publish_backend_result(
  const VioBackendResult & backend_result,
  const builtin_interfaces::msg::Time & stamp)
{
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = this->odom_frame_id_;
  odom_msg.child_frame_id = this->base_frame_id_;
  fill_pose_msg(odom_msg.pose.pose, backend_result.odom_T_base);
  odom_msg.twist.twist.linear.x = backend_result.velocity_odom.x();
  odom_msg.twist.twist.linear.y = backend_result.velocity_odom.y();
  odom_msg.twist.twist.linear.z = backend_result.velocity_odom.z();
  this->odom_pub_->publish(odom_msg);

  geometry_msgs::msg::TransformStamped odom_tf_msg;
  odom_tf_msg.header.stamp = stamp;
  odom_tf_msg.header.frame_id = this->odom_frame_id_;
  odom_tf_msg.child_frame_id = this->base_frame_id_;
  fill_transform_msg(odom_tf_msg.transform, backend_result.odom_T_base);
  this->odom_tf_broadcaster_->sendTransform(odom_tf_msg);
}

void VioNode::publish_stereo_observations(
  const VioFrontendResult & frontend_result,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id)
{
  stereo_vio_msgs::msg::StereoObservations observations_msg;
  observations_msg.header.stamp = stamp;
  observations_msg.header.frame_id = frame_id;

  observations_msg.feature_ids.reserve(frontend_result.observations.size());
  observations_msg.left_u.reserve(frontend_result.observations.size());
  observations_msg.left_v.reserve(frontend_result.observations.size());
  observations_msg.right_u.reserve(frontend_result.observations.size());
  observations_msg.right_v.reserve(frontend_result.observations.size());
  observations_msg.landmarks_left.reserve(frontend_result.observations.size());

  for (const VioFeatureObservation & observation : frontend_result.observations) {
    if (!is_observation_publishable(observation)) {
      continue;
    }

    observations_msg.feature_ids.push_back(observation.id);
    observations_msg.left_u.push_back(observation.left_pixel.x);
    observations_msg.left_v.push_back(observation.left_pixel.y);
    observations_msg.right_u.push_back(observation.right_pixel.x);
    observations_msg.right_v.push_back(observation.right_pixel.y);
    observations_msg.landmarks_left.push_back(point32_from_cv_point(observation.landmark_left));
  }

  this->observations_pub_->publish(observations_msg);
}

}  // namespace stereo_vio

RCLCPP_COMPONENTS_REGISTER_NODE(stereo_vio::VioNode)
