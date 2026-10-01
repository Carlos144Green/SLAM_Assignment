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

#include "rerun_wrapper/rerun_bridge.hpp"

#include "rerun_wrapper/rerun_conversions.hpp"
#include "rerun_wrapper/rerun_observations.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace rerun_wrapper
{

namespace
{

constexpr char kRosBaseFrameId[] = "base_link";
constexpr char kRosImuFrameId[] = "oak_imu_link";
constexpr char kRosLeftCameraFrameId[] = "oak_left_camera_frame";
constexpr char kRosRightCameraFrameId[] = "oak_right_camera_frame";
constexpr char kRosLeftOpticalFrameId[] = "oak_left_optical_frame";
constexpr char kRosRightOpticalFrameId[] = "oak_right_optical_frame";

constexpr char kRerunApplicationId[] = "rerun_wrapper";
constexpr char kRerunRecordingId[] = "stereo_vio_rerun";
constexpr char kRosTimeTimeline[] = "ros_time";
constexpr char kWorldPath[] = "world";

constexpr char kGroundTruthBasePath[] = "world/gt_odom/gt_base_link";
constexpr char kGroundTruthImuPath[] = "world/gt_odom/gt_base_link/oak_imu_link";
constexpr char kGroundTruthLeftCameraPath[] = "world/gt_odom/gt_base_link/oak_left_camera_frame";
constexpr char kGroundTruthRightCameraPath[] = "world/gt_odom/gt_base_link/oak_right_camera_frame";
constexpr char kGroundTruthLeftOpticalPath[] =
  "world/gt_odom/gt_base_link/oak_left_camera_frame/oak_left_optical_frame";
constexpr char kGroundTruthRightOpticalPath[] =
  "world/gt_odom/gt_base_link/oak_right_camera_frame/oak_right_optical_frame";
constexpr char kGroundTruthTrajectorySegmentPathPrefix[] =
  "world/gt_odom/ground_truth_trajectory/segments/";
constexpr char kGroundTruthXyTrajectorySegmentPathPrefix[] =
  "odom_xy/ground_truth/segments/";

constexpr char kStereoVioBasePath[] = "world/stereo_vio_odom/base_link";
constexpr char kStereoVioImuPath[] = "world/stereo_vio_odom/base_link/oak_imu_link";
constexpr char kStereoVioLeftCameraPath[] = "world/stereo_vio_odom/base_link/oak_left_camera_frame";
constexpr char kStereoVioRightCameraPath[] =
  "world/stereo_vio_odom/base_link/oak_right_camera_frame";
constexpr char kStereoVioLeftOpticalPath[] =
  "world/stereo_vio_odom/base_link/oak_left_camera_frame/oak_left_optical_frame";
constexpr char kStereoVioRightOpticalPath[] =
  "world/stereo_vio_odom/base_link/oak_right_camera_frame/oak_right_optical_frame";
constexpr char kStereoVioLeftLandmarksOpticalPath[] =
  "world/stereo_vio_odom/base_link/oak_left_camera_frame/oak_left_landmarks_optical_frame";
constexpr char kStereoVioTrajectorySegmentPathPrefix[] =
  "world/stereo_vio_odom/stereo_vio_trajectory/segments/";
constexpr char kStereoVioXyTrajectorySegmentPathPrefix[] =
  "odom_xy/stereo_vio_aligned/segments/";

constexpr char kGroundTruthLeftImagePath[] =
  "world/gt_odom/gt_base_link/oak_left_camera_frame/oak_left_optical_frame/image";
constexpr char kGroundTruthRightImagePath[] =
  "world/gt_odom/gt_base_link/oak_right_camera_frame/oak_right_optical_frame/image";
constexpr char kStereoVioLeftImagePath[] =
  "world/stereo_vio_odom/base_link/oak_left_camera_frame/oak_left_optical_frame/image";
constexpr char kStereoVioRightImagePath[] =
  "world/stereo_vio_odom/base_link/oak_right_camera_frame/oak_right_optical_frame/image";
constexpr char kStereoVioLeftObservationsPath[] =
  "world/stereo_vio_odom/base_link/oak_left_camera_frame/oak_left_optical_frame/image/observations";
constexpr char kStereoVioRightObservationsPath[] =
  "world/stereo_vio_odom/base_link/oak_right_camera_frame/oak_right_optical_frame/image/observations";
constexpr char kStereoVioLandmarksLeftPath[] =
  "world/stereo_vio_odom/base_link/oak_left_camera_frame/oak_left_landmarks_optical_frame/landmarks_left";
constexpr char kErrorAtePath[] = "errors/ate_m";
constexpr char kErrorArePath[] = "errors/are_deg";
constexpr char kErrorRpeTranslationPath[] = "errors/rpe_translation_m";
constexpr char kErrorRpeRotationPath[] = "errors/rpe_rotation_deg";
constexpr char kErrorDistancePath[] = "errors/distance_m";

constexpr float kTrajectoryMinDistanceMeters = 0.02F;
constexpr float kTrajectoryMinSquaredDistanceMeters =
  kTrajectoryMinDistanceMeters * kTrajectoryMinDistanceMeters;
constexpr uint8_t kGroundTruthTrajectoryAlpha = 128U;
constexpr uint8_t kGroundTruthCameraAlpha = 96U;
constexpr float kGroundTruthImageOpacity = 0.45F;

[[nodiscard]] rerun::Color ground_truth_camera_color()
{
  return rerun::Color{52, 120, 219, kGroundTruthCameraAlpha};
}

[[nodiscard]] rerun::Color ground_truth_trajectory_color()
{
  return rerun::Color{52, 120, 219, kGroundTruthTrajectoryAlpha};
}

[[nodiscard]] rerun::Color ground_truth_xy_trajectory_color()
{
  return rerun::Color{52, 120, 219};
}

[[nodiscard]] rerun::Color stereo_vio_trajectory_color()
{
  return rerun::Color{231, 76, 60};
}

[[nodiscard]] rerun::Color stereo_vio_feature_color()
{
  return rerun::Color{255, 0, 0};
}

[[nodiscard]] rerun::SeriesLines xyz_series_lines()
{
  return rerun::SeriesLines()
    .with_names(std::array<rerun::components::Name, 3>{
      rerun::components::Name{"x"},
      rerun::components::Name{"y"},
      rerun::components::Name{"z"}})
    .with_colors(std::array<rerun::Color, 3>{
      rerun::Color{231, 76, 60},
      rerun::Color{39, 174, 96},
      rerun::Color{52, 120, 219}});
}

[[nodiscard]] rerun::SeriesLines current_and_rmse_series_lines()
{
  return rerun::SeriesLines()
    .with_names(std::array<rerun::components::Name, 2>{
      rerun::components::Name{"Current"},
      rerun::components::Name{"RMSE"}});
}

[[nodiscard]] rerun::SeriesLines distance_error_series_lines()
{
  return rerun::SeriesLines()
    .with_names(std::array<rerun::components::Name, 2>{
      rerun::components::Name{"Ground truth"},
      rerun::components::Name{"Stereo VIO"}})
    .with_colors(std::array<rerun::Color, 2>{
      rerun::Color{52, 120, 219},
      rerun::Color{231, 76, 60}});
}

[[nodiscard]] Eigen::Isometry3d odometry_pose_to_isometry(
  const nav_msgs::msg::Odometry & odometry)
{
  const geometry_msgs::msg::Point & position = odometry.pose.pose.position;
  const geometry_msgs::msg::Quaternion & orientation = odometry.pose.pose.orientation;

  Eigen::Quaterniond rotation(orientation.w, orientation.x, orientation.y, orientation.z);
  if (rotation.norm() == 0.0) {
    throw std::invalid_argument("Odometry pose has a zero-norm quaternion");
  }
  rotation.normalize();

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = rotation.toRotationMatrix();
  pose.translation() = Eigen::Vector3d(position.x, position.y, position.z);
  return pose;
}

[[nodiscard]] rerun::datatypes::Vec2D xy_trajectory_point_from_pose(
  const Eigen::Isometry3d & pose)
{
  const Eigen::Vector3d translation = pose.translation();
  return rerun::datatypes::Vec2D{
    static_cast<float>(translation.x()),
    static_cast<float>(translation.y())};
}

[[nodiscard]] float squared_distance_xy(
  const rerun::datatypes::Vec2D & first,
  const rerun::datatypes::Vec2D & second)
{
  const float dx = first.x() - second.x();
  const float dy = first.y() - second.y();
  return dx * dx + dy * dy;
}

}  // namespace

RerunBridge::RerunBridge(const std::string & blueprint_path)
: recording_(kRerunApplicationId, kRerunRecordingId),
  left_camera_logged_(false),
  right_camera_logged_(false),
  imu_transform_logged_(false),
  left_camera_transform_logged_(false),
  right_camera_transform_logged_(false),
  left_optical_transform_logged_(false),
  right_optical_transform_logged_(false),
  ground_truth_trajectory_segment_count_(0U),
  stereo_vio_trajectory_segment_count_(0U),
  ground_truth_xy_trajectory_segment_count_(0U),
  stereo_vio_xy_trajectory_segment_count_(0U),
  next_keypoint_id_(1U)
{
  rerun::SpawnOptions spawn_options;
  spawn_options.memory_limit = "4GiB";
  spawn_options.server_memory_limit = "4GiB";
  recording_.set_global();
  recording_.spawn(spawn_options).exit_on_failure();
  if (!blueprint_path.empty()) {
    recording_.log_file_from_path(std::filesystem::path{blueprint_path});
    recording_.flush_blocking(2.0F).exit_on_failure();
  }
  log_world_view_coordinates();
  log_imu_series_styles();
  log_error_series_styles();
}

void RerunBridge::set_time(const builtin_interfaces::msg::Time & stamp)
{
  recording_.set_time_timestamp_nanos_since_epoch(kRosTimeTimeline, stamp_to_ns(stamp));
}

void RerunBridge::log_left_camera_info_once(const sensor_msgs::msg::CameraInfo & left)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (left_camera_logged_) {
    return;
  }
  recording_.log_static(
    kGroundTruthLeftOpticalPath,
    pinhole_from_camera_info(left).with_color(ground_truth_camera_color()));
  recording_.log_static(kStereoVioLeftOpticalPath, pinhole_from_camera_info(left));
  left_camera_logged_ = true;
}

void RerunBridge::log_right_camera_info_once(const sensor_msgs::msg::CameraInfo & right)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (right_camera_logged_) {
    return;
  }
  recording_.log_static(
    kGroundTruthRightOpticalPath,
    pinhole_from_camera_info(right).with_color(ground_truth_camera_color()));
  recording_.log_static(kStereoVioRightOpticalPath, pinhole_from_camera_info(right));
  right_camera_logged_ = true;
}

void RerunBridge::log_synced_frame(
  const sensor_msgs::msg::Image & left,
  const sensor_msgs::msg::Image & right,
  const nav_msgs::msg::Odometry & ground_truth_odom,
  const nav_msgs::msg::Odometry & stereo_vio_odom,
  const stereo_vio_msgs::msg::StereoObservations & stereo_vio_observations)
{
  const std::vector<uint8_t> left_jpeg = encode_jpeg(left);
  const std::vector<uint8_t> right_jpeg = encode_jpeg(right);
  const std::vector<rerun::Position2D> stereo_vio_left_observations =
    make_left_observation_points(stereo_vio_observations);
  const std::vector<rerun::Position2D> stereo_vio_right_observations =
    make_right_observation_points(stereo_vio_observations);
  const std::vector<rerun::Position3D> stereo_vio_landmarks_left =
    make_landmark_points_left(stereo_vio_observations);
  const rerun::Color feature_color = stereo_vio_feature_color();
  const rerun::EncodedImage left_encoded_image = rerun::EncodedImage::from_bytes(
    rerun::borrow(left_jpeg),
    rerun::components::MediaType::jpeg());
  const rerun::EncodedImage right_encoded_image = rerun::EncodedImage::from_bytes(
    rerun::borrow(right_jpeg),
    rerun::components::MediaType::jpeg());
  const rerun::EncodedImage ground_truth_left_encoded_image =
    rerun::EncodedImage::from_bytes(
      rerun::borrow(left_jpeg),
      rerun::components::MediaType::jpeg())
    .with_opacity(kGroundTruthImageOpacity);
  const rerun::EncodedImage ground_truth_right_encoded_image =
    rerun::EncodedImage::from_bytes(
      rerun::borrow(right_jpeg),
      rerun::components::MediaType::jpeg())
    .with_opacity(kGroundTruthImageOpacity);
  const Eigen::Isometry3d ground_truth_pose = odometry_pose_to_isometry(ground_truth_odom);
  const Eigen::Isometry3d stereo_vio_pose = odometry_pose_to_isometry(stereo_vio_odom);

  std::lock_guard<std::mutex> lock(mutex_);
  recording_.set_time_timestamp_nanos_since_epoch(
    kRosTimeTimeline,
    stamp_to_ns(left.header.stamp));
  const std::vector<rerun::components::KeypointId> stereo_vio_keypoint_ids =
    make_rerun_keypoint_ids(stereo_vio_observations);
  recording_.log(kGroundTruthBasePath, transform_from_odom(ground_truth_odom));
  recording_.log(kStereoVioBasePath, transform_from_odom(stereo_vio_odom));

  recording_.log(kGroundTruthLeftImagePath, ground_truth_left_encoded_image);
  recording_.log(kGroundTruthRightImagePath, ground_truth_right_encoded_image);
  recording_.log(kStereoVioLeftImagePath, left_encoded_image);
  recording_.log(kStereoVioRightImagePath, right_encoded_image);
  recording_.log(
    kStereoVioLeftObservationsPath,
    rerun::Points2D(stereo_vio_left_observations)
      .with_colors(feature_color)
      .with_keypoint_ids(stereo_vio_keypoint_ids));
  recording_.log(
    kStereoVioRightObservationsPath,
    rerun::Points2D(stereo_vio_right_observations)
      .with_colors(feature_color)
      .with_keypoint_ids(stereo_vio_keypoint_ids));
  recording_.log(
    kStereoVioLandmarksLeftPath,
    rerun::Points3D(stereo_vio_landmarks_left)
      .with_colors(feature_color)
      .with_keypoint_ids(stereo_vio_keypoint_ids));

  log_trajectory_sample(
    kGroundTruthTrajectorySegmentPathPrefix,
    trajectory_point_from_odom(ground_truth_odom),
    ground_truth_trajectory_color(),
    last_ground_truth_trajectory_point_,
    ground_truth_trajectory_segment_count_);
  log_trajectory_sample(
    kStereoVioTrajectorySegmentPathPrefix,
    trajectory_point_from_odom(stereo_vio_odom),
    stereo_vio_trajectory_color(),
    last_stereo_vio_trajectory_point_,
    stereo_vio_trajectory_segment_count_);

  if (!alignment_gt_from_vio_.has_value()) {
    alignment_gt_from_vio_ = ground_truth_pose * stereo_vio_pose.inverse();
  }
  const Eigen::Isometry3d aligned_stereo_vio_pose =
    alignment_gt_from_vio_.value() * stereo_vio_pose;
  log_xy_trajectory_sample(
    kGroundTruthXyTrajectorySegmentPathPrefix,
    xy_trajectory_point_from_pose(ground_truth_pose),
    ground_truth_xy_trajectory_color(),
    last_ground_truth_xy_trajectory_point_,
    ground_truth_xy_trajectory_segment_count_);
  log_xy_trajectory_sample(
    kStereoVioXyTrajectorySegmentPathPrefix,
    xy_trajectory_point_from_pose(aligned_stereo_vio_pose),
    stereo_vio_trajectory_color(),
    last_stereo_vio_xy_trajectory_point_,
    stereo_vio_xy_trajectory_segment_count_);
}

void RerunBridge::log_imu(const sensor_msgs::msg::Imu & imu)
{
  recording_.set_time_timestamp_nanos_since_epoch(
    kRosTimeTimeline,
    stamp_to_ns(imu.header.stamp));
  recording_.log(
    "gyroscope",
    rerun::archetypes::Scalars({
      imu.angular_velocity.x,
      imu.angular_velocity.y,
      imu.angular_velocity.z}));
  recording_.log(
    "accelerometer",
    rerun::archetypes::Scalars({
      imu.linear_acceleration.x,
      imu.linear_acceleration.y,
      imu.linear_acceleration.z}));
}

void RerunBridge::log_odometry_error(const stereo_vio_msgs::msg::OdometryError & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  recording_.set_time_timestamp_nanos_since_epoch(
    kRosTimeTimeline,
    stamp_to_ns(error.header.stamp));
  recording_.log(
    kErrorAtePath,
    rerun::archetypes::Scalars({
      error.ate_m,
      error.ate_rmse_m}));
  recording_.log(
    kErrorArePath,
    rerun::archetypes::Scalars({
      error.are_deg,
      error.are_rmse_deg}));
  recording_.log(
    kErrorRpeTranslationPath,
    rerun::archetypes::Scalars({
      error.rpe_translation_m,
      error.rpe_translation_rmse_m}));
  recording_.log(
    kErrorRpeRotationPath,
    rerun::archetypes::Scalars({
      error.rpe_rotation_deg,
      error.rpe_rotation_rmse_deg}));
  recording_.log(
    kErrorDistancePath,
    rerun::archetypes::Scalars({
      error.gt_distance_m,
      error.vio_distance_m}));
}

void RerunBridge::log_static_transforms(const tf2_msgs::msg::TFMessage & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const geometry_msgs::msg::TransformStamped & transform : msg.transforms) {
    log_static_transform(transform);
  }
}

void RerunBridge::log_world_view_coordinates()
{
  recording_.log_static(kWorldPath, rerun::ViewCoordinates::FLU);
}

void RerunBridge::log_imu_series_styles()
{
  const rerun::SeriesLines series_lines = xyz_series_lines();
  recording_.log_static("gyroscope", series_lines);
  recording_.log_static("accelerometer", series_lines);
}

void RerunBridge::log_error_series_styles()
{
  const rerun::SeriesLines current_and_rmse = current_and_rmse_series_lines();
  recording_.log_static(kErrorAtePath, current_and_rmse);
  recording_.log_static(kErrorArePath, current_and_rmse);
  recording_.log_static(kErrorRpeTranslationPath, current_and_rmse);
  recording_.log_static(kErrorRpeRotationPath, current_and_rmse);
  recording_.log_static(kErrorDistancePath, distance_error_series_lines());
}

void RerunBridge::log_static_transform(const geometry_msgs::msg::TransformStamped & transform)
{
  if (transform.header.frame_id == kRosBaseFrameId &&
    transform.child_frame_id == kRosImuFrameId && !imu_transform_logged_)
  {
    recording_.log_static(kGroundTruthImuPath, transform_from_tf(transform));
    recording_.log_static(kStereoVioImuPath, transform_from_tf(transform));
    imu_transform_logged_ = true;
    return;
  }

  if (transform.header.frame_id == kRosBaseFrameId &&
    transform.child_frame_id == kRosLeftCameraFrameId && !left_camera_transform_logged_)
  {
    recording_.log_static(kGroundTruthLeftCameraPath, transform_from_tf(transform));
    recording_.log_static(kStereoVioLeftCameraPath, transform_from_tf(transform));
    left_camera_transform_logged_ = true;
    return;
  }

  if (transform.header.frame_id == kRosBaseFrameId &&
    transform.child_frame_id == kRosRightCameraFrameId && !right_camera_transform_logged_)
  {
    recording_.log_static(kGroundTruthRightCameraPath, transform_from_tf(transform));
    recording_.log_static(kStereoVioRightCameraPath, transform_from_tf(transform));
    right_camera_transform_logged_ = true;
    return;
  }

  if (transform.header.frame_id == kRosLeftCameraFrameId &&
    transform.child_frame_id == kRosLeftOpticalFrameId && !left_optical_transform_logged_)
  {
    recording_.log_static(kGroundTruthLeftOpticalPath, transform_from_tf(transform));
    recording_.log_static(kStereoVioLeftOpticalPath, transform_from_tf(transform));
    recording_.log_static(kStereoVioLeftLandmarksOpticalPath, transform_from_tf(transform));
    left_optical_transform_logged_ = true;
    return;
  }

  if (transform.header.frame_id == kRosRightCameraFrameId &&
    transform.child_frame_id == kRosRightOpticalFrameId && !right_optical_transform_logged_)
  {
    recording_.log_static(kGroundTruthRightOpticalPath, transform_from_tf(transform));
    recording_.log_static(kStereoVioRightOpticalPath, transform_from_tf(transform));
    right_optical_transform_logged_ = true;
    return;
  }
}

uint16_t RerunBridge::rerun_keypoint_id_from_feature_id(const uint64_t feature_id)
{
  const std::unordered_map<uint64_t, uint16_t>::const_iterator existing =
    feature_id_to_keypoint_id_.find(feature_id);
  if (existing != feature_id_to_keypoint_id_.end()) {
    return existing->second;
  }

  const uint16_t keypoint_id = next_keypoint_id_;
  feature_id_to_keypoint_id_.emplace(feature_id, keypoint_id);
  if (next_keypoint_id_ == std::numeric_limits<uint16_t>::max()) {
    next_keypoint_id_ = 1U;
  } else {
    ++next_keypoint_id_;
  }
  return keypoint_id;
}

std::vector<rerun::components::KeypointId> RerunBridge::make_rerun_keypoint_ids(
  const stereo_vio_msgs::msg::StereoObservations & observations)
{
  const std::size_t observation_count = stereo_observation_count(observations);
  std::vector<rerun::components::KeypointId> keypoint_ids;
  keypoint_ids.reserve(observation_count);

  for (std::size_t index = 0U; index < observation_count; ++index) {
    keypoint_ids.emplace_back(rerun_keypoint_id_from_feature_id(observations.feature_ids[index]));
  }

  return keypoint_ids;
}

void RerunBridge::log_trajectory_sample(
  const char * path_prefix,
  const rerun::datatypes::Vec3D & current_point,
  const rerun::Color & color,
  std::optional<rerun::datatypes::Vec3D> & last_point,
  uint64_t & segment_count)
{
  if (!last_point.has_value()) {
    last_point = current_point;
    return;
  }

  if (squared_distance(last_point.value(), current_point) < kTrajectoryMinSquaredDistanceMeters) {
    return;
  }

  const std::array<rerun::datatypes::Vec3D, 2> segment_points{last_point.value(), current_point};
  const rerun::components::LineStrip3D trajectory_segment{segment_points};
  const std::array<rerun::Color, 1> trajectory_colors{color};
  const rerun::LineStrips3D colored_trajectory_segment =
    rerun::LineStrips3D(trajectory_segment).with_colors(trajectory_colors);
  const std::string segment_path = std::string(path_prefix) + std::to_string(segment_count);
  recording_.log_static(segment_path, colored_trajectory_segment);
  last_point = current_point;
  ++segment_count;
}

void RerunBridge::log_xy_trajectory_sample(
  const char * path_prefix,
  const rerun::datatypes::Vec2D & current_point,
  const rerun::Color & color,
  std::optional<rerun::datatypes::Vec2D> & last_point,
  uint64_t & segment_count)
{
  if (!last_point.has_value()) {
    last_point = current_point;
    return;
  }

  if (squared_distance_xy(last_point.value(), current_point) <
    kTrajectoryMinSquaredDistanceMeters)
  {
    return;
  }

  const std::array<rerun::datatypes::Vec2D, 2> segment_points{
    last_point.value(),
    current_point};
  const rerun::components::LineStrip2D trajectory_segment{segment_points};
  const std::array<rerun::Color, 1> trajectory_colors{color};
  const rerun::LineStrips2D colored_trajectory_segment =
    rerun::LineStrips2D(trajectory_segment).with_colors(trajectory_colors);
  const std::string segment_path = std::string(path_prefix) + std::to_string(segment_count);
  recording_.log_static(segment_path, colored_trajectory_segment);
  last_point = current_point;
  ++segment_count;
}

}  // namespace rerun_wrapper
