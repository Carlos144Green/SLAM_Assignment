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

#include "stereo_vio/vio_frontend.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace stereo_vio
{

struct VioImuSample
{
  double stamp_seconds = 0.0;
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};

struct VioBackendFrameTransforms
{
  Eigen::Isometry3d imu_T_base = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d imu_T_left_camera = Eigen::Isometry3d::Identity();
};

struct VioBackendOptions
{
  double gravity_magnitude = 9.81;
  bool use_initial_gravity_alignment = true;
  int gravity_alignment_minimum_samples = 5;
  double gravity_alignment_acceleration_tolerance = 2.0;
  double gravity_alignment_max_angular_velocity = 0.35;
  bool use_planar_3dof_mode = true;
  double accelerometer_noise_sigma = 0.11;
  double gyroscope_noise_sigma = 0.01;
  double accelerometer_bias_random_walk_sigma = 0.0005;
  double gyroscope_bias_random_walk_sigma = 0.0001;
  double integration_noise_sigma = 1.0e-7;
  double visual_huber_k = 1.345;
  double visual_between_rotation_sigma = 0.05;
  double visual_between_translation_sigma = 0.05;
  double visual_3d_ransac_threshold = 0.15;
  double pose_prior_rotation_sigma = 1.0e-3;
  double pose_prior_translation_sigma = 1.0e-3;
  double pose_prediction_rotation_sigma = 0.75;
  double pose_prediction_translation_sigma = 3.0;
  double velocity_prior_sigma = 1.0e-2;
  double velocity_prediction_sigma = 3.0;
  double bias_prior_accelerometer_sigma = 1.0e-3;
  double bias_prior_gyroscope_sigma = 1.0e-4;
  double bias_prediction_accelerometer_sigma = 0.05;
  double bias_prediction_gyroscope_sigma = 0.01;
  int max_visual_observations_per_frame = 150;
  int minimum_visual_track_age = 2;
  int minimum_visual_odometry_inliers = 12;
  double isam2_relinearize_threshold = 0.1;
  int isam2_relinearize_skip = 1;
};

struct VioBackendResult
{
  bool valid = false;
  bool initialized = false;
  uint64_t frame_id = 0U;
  double stamp_seconds = 0.0;
  Eigen::Isometry3d odom_T_base = Eigen::Isometry3d::Identity();
  Eigen::Vector3d velocity_odom = Eigen::Vector3d::Zero();
  std::string status_message;
};

class VioBackend final
{
public:
  VioBackend();
  explicit VioBackend(const VioBackendOptions & options);
  ~VioBackend() = default;

  VioBackend(const VioBackend &) = delete;
  VioBackend & operator=(const VioBackend &) = delete;
  VioBackend(VioBackend &&) = delete;
  VioBackend & operator=(VioBackend &&) = delete;

  [[nodiscard]] VioBackendResult process_frame(
    double stamp_seconds,
    const VioFrontendResult & frontend_result,
    const std::vector<VioImuSample> & imu_samples,
    const VioBackendFrameTransforms & frame_transforms);

private:
  struct BackendFrame
  {
    uint64_t id = 0U;
    double stamp_seconds = 0.0;
    std::vector<VioFeatureObservation> observations;
    std::vector<VioImuSample> imu_samples;
    gtsam::Pose3 odom_T_imu;
    gtsam::Vector3 velocity_odom = gtsam::Vector3::Zero();
    gtsam::imuBias::ConstantBias imu_bias;
    std::string status_message;
  };

  [[nodiscard]] bool is_options_valid() const;
  [[nodiscard]] std::vector<VioFeatureObservation> make_backend_observations(
    const VioFrontendResult & frontend_result) const;
  [[nodiscard]] BackendFrame make_first_frame(
    double stamp_seconds,
    const VioFrontendResult & frontend_result,
    const std::vector<VioImuSample> & imu_samples,
    const VioBackendFrameTransforms & frame_transforms) const;
  [[nodiscard]] BackendFrame make_next_frame(
    double stamp_seconds,
    const VioFrontendResult & frontend_result,
    const std::vector<VioImuSample> & imu_samples) const;
  [[nodiscard]] VioBackendResult make_result(
    const BackendFrame & frame,
    const VioBackendFrameTransforms & frame_transforms,
    const std::string & status_message) const;
  [[nodiscard]] bool update_isam2(
    BackendFrame & frame,
    const VioBackendFrameTransforms & frame_transforms,
    VioBackendResult & result);

  [[nodiscard]] bool reset_isam2_at_frame(
    const BackendFrame & frame,
    const VioBackendFrameTransforms & frame_transforms,
    const std::string & status_message,
    VioBackendResult & result);

  [[nodiscard]] std::optional<Eigen::Isometry3d> estimate_initial_odom_T_base_from_gravity(
    const std::vector<VioImuSample> & imu_samples,
    const VioBackendFrameTransforms & frame_transforms,
    std::string & status_message) const;

  VioBackendOptions options_;
  bool initialized_;
  uint64_t next_frame_id_;
  std::vector<VioImuSample> initial_gravity_samples_;
  BackendFrame previous_frame_;
  std::unique_ptr<gtsam::ISAM2> isam2_;
};

}  // namespace stereo_vio
