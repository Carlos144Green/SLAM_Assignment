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

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "stereo_vio_msgs/msg/odometry_error.hpp"
#include "stereo_vio_msgs/msg/stereo_observations.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include <Eigen/Geometry>

#include <rerun.hpp>

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace rerun_wrapper
{

class RerunBridge final
{
public:
  explicit RerunBridge(const std::string & blueprint_path);
  ~RerunBridge() = default;

  RerunBridge(const RerunBridge &) = delete;
  RerunBridge & operator=(const RerunBridge &) = delete;
  RerunBridge(RerunBridge &&) = delete;
  RerunBridge & operator=(RerunBridge &&) = delete;

  void set_time(const builtin_interfaces::msg::Time & stamp);
  void log_left_camera_info_once(const sensor_msgs::msg::CameraInfo & left);
  void log_right_camera_info_once(const sensor_msgs::msg::CameraInfo & right);
  void log_synced_frame(
    const sensor_msgs::msg::Image & left,
    const sensor_msgs::msg::Image & right,
    const nav_msgs::msg::Odometry & ground_truth_odom,
    const nav_msgs::msg::Odometry & stereo_vio_odom,
    const stereo_vio_msgs::msg::StereoObservations & stereo_vio_observations);
  void log_imu(const sensor_msgs::msg::Imu & imu);
  void log_odometry_error(const stereo_vio_msgs::msg::OdometryError & error);
  void log_static_transforms(const tf2_msgs::msg::TFMessage & msg);

private:
  void log_world_view_coordinates();
  void log_imu_series_styles();
  void log_error_series_styles();
  void log_static_transform(const geometry_msgs::msg::TransformStamped & transform);

  [[nodiscard]] uint16_t rerun_keypoint_id_from_feature_id(uint64_t feature_id);
  [[nodiscard]] std::vector<rerun::components::KeypointId> make_rerun_keypoint_ids(
    const stereo_vio_msgs::msg::StereoObservations & observations);
  void log_trajectory_sample(
    const char * path_prefix,
    const rerun::datatypes::Vec3D & current_point,
    const rerun::Color & color,
    std::optional<rerun::datatypes::Vec3D> & last_point,
    uint64_t & segment_count);
  void log_xy_trajectory_sample(
    const char * path_prefix,
    const rerun::datatypes::Vec2D & current_point,
    const rerun::Color & color,
    std::optional<rerun::datatypes::Vec2D> & last_point,
    uint64_t & segment_count);

  rerun::RecordingStream recording_;
  bool left_camera_logged_;
  bool right_camera_logged_;
  bool imu_transform_logged_;
  bool left_camera_transform_logged_;
  bool right_camera_transform_logged_;
  bool left_optical_transform_logged_;
  bool right_optical_transform_logged_;
  std::optional<rerun::datatypes::Vec3D> last_ground_truth_trajectory_point_;
  std::optional<rerun::datatypes::Vec3D> last_stereo_vio_trajectory_point_;
  std::optional<Eigen::Isometry3d> alignment_gt_from_vio_;
  std::optional<rerun::datatypes::Vec2D> last_ground_truth_xy_trajectory_point_;
  std::optional<rerun::datatypes::Vec2D> last_stereo_vio_xy_trajectory_point_;
  uint64_t ground_truth_trajectory_segment_count_;
  uint64_t stereo_vio_trajectory_segment_count_;
  uint64_t ground_truth_xy_trajectory_segment_count_;
  uint64_t stereo_vio_xy_trajectory_segment_count_;
  std::unordered_map<uint64_t, uint16_t> feature_id_to_keypoint_id_;
  uint16_t next_keypoint_id_;
  std::mutex mutex_;
};

}  // namespace rerun_wrapper
