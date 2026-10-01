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

#include "stereo_vio/vio_backend.hpp"

#include <Eigen/SVD>

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

constexpr std::size_t kMaxInitialGravityAlignmentSamples = 1000U;

struct VisualCorrespondence
{
  Eigen::Vector3d previous_left_point = Eigen::Vector3d::Zero();
  Eigen::Vector3d current_left_point = Eigen::Vector3d::Zero();
};

struct RigidTransformEstimate
{
  bool valid = false;
  Eigen::Isometry3d previous_left_T_current_left = Eigen::Isometry3d::Identity();
  std::vector<std::size_t> inlier_indices;
  double mean_error = std::numeric_limits<double>::max();
};

struct VisualOdometryMeasurement
{
  bool valid = false;
  gtsam::Pose3 previous_imu_T_current_imu;
};

[[nodiscard]] gtsam::Key make_pose_key(const uint64_t frame_id)
{
  return gtsam::Symbol('x', frame_id);
}

[[nodiscard]] gtsam::Key make_velocity_key(const uint64_t frame_id)
{
  return gtsam::Symbol('v', frame_id);
}

[[nodiscard]] gtsam::Key make_bias_key(const uint64_t frame_id)
{
  return gtsam::Symbol('b', frame_id);
}

[[nodiscard]] Eigen::Vector3d make_vector3d(const cv::Point3d & point)
{
  return Eigen::Vector3d(point.x, point.y, point.z);
}

[[nodiscard]] gtsam::Pose3 make_pose3(const Eigen::Isometry3d & transform)
{
  return gtsam::Pose3(
    gtsam::Rot3(transform.linear()),
    gtsam::Point3(
      transform.translation().x(),
      transform.translation().y(),
      transform.translation().z()));
}

[[nodiscard]] Eigen::Isometry3d make_isometry3d(const gtsam::Pose3 & pose)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = pose.rotation().matrix();
  transform.translation() = Eigen::Vector3d(
    pose.translation().x(),
    pose.translation().y(),
    pose.translation().z());
  return transform;
}

[[nodiscard]] Eigen::Isometry3d make_planar_3dof_transform(
  const Eigen::Isometry3d & transform)
{
  const double yaw = std::atan2(transform.linear()(1, 0), transform.linear()(0, 0));

  Eigen::Isometry3d planar_transform = Eigen::Isometry3d::Identity();
  planar_transform.linear() =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  planar_transform.translation() =
    Eigen::Vector3d(transform.translation().x(), transform.translation().y(), 0.0);
  return planar_transform;
}

[[nodiscard]] gtsam::Vector3 make_vector3(const Eigen::Vector3d & vector)
{
  gtsam::Vector3 gtsam_vector;
  gtsam_vector << vector.x(), vector.y(), vector.z();
  return gtsam_vector;
}

[[nodiscard]] gtsam::ISAM2Params make_isam2_params(
  const stereo_vio::VioBackendOptions & options)
{
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = options.isam2_relinearize_threshold;
  params.relinearizeSkip = options.isam2_relinearize_skip;
  params.cacheLinearizedFactors = true;
  return params;
}

[[nodiscard]] gtsam::SharedNoiseModel make_pose_prior_noise(
  const stereo_vio::VioBackendOptions & options)
{
  gtsam::Vector sigmas(6);
  sigmas << options.pose_prior_rotation_sigma,
    options.pose_prior_rotation_sigma,
    options.pose_prior_rotation_sigma,
    options.pose_prior_translation_sigma,
    options.pose_prior_translation_sigma,
    options.pose_prior_translation_sigma;
  return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

[[nodiscard]] gtsam::SharedNoiseModel make_pose_prediction_noise(
  const stereo_vio::VioBackendOptions & options)
{
  gtsam::Vector sigmas(6);
  sigmas << options.pose_prediction_rotation_sigma,
    options.pose_prediction_rotation_sigma,
    options.pose_prediction_rotation_sigma,
    options.pose_prediction_translation_sigma,
    options.pose_prediction_translation_sigma,
    options.pose_prediction_translation_sigma;
  return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

[[nodiscard]] gtsam::SharedNoiseModel make_velocity_prior_noise(
  const stereo_vio::VioBackendOptions & options)
{
  return gtsam::noiseModel::Isotropic::Sigma(3, options.velocity_prior_sigma);
}

[[nodiscard]] gtsam::SharedNoiseModel make_velocity_prediction_noise(
  const stereo_vio::VioBackendOptions & options)
{
  return gtsam::noiseModel::Isotropic::Sigma(3, options.velocity_prediction_sigma);
}

[[nodiscard]] gtsam::SharedNoiseModel make_bias_prior_noise(
  const stereo_vio::VioBackendOptions & options)
{
  gtsam::Vector sigmas(6);
  sigmas << options.bias_prior_accelerometer_sigma,
    options.bias_prior_accelerometer_sigma,
    options.bias_prior_accelerometer_sigma,
    options.bias_prior_gyroscope_sigma,
    options.bias_prior_gyroscope_sigma,
    options.bias_prior_gyroscope_sigma;
  return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

[[nodiscard]] gtsam::SharedNoiseModel make_bias_prediction_noise(
  const stereo_vio::VioBackendOptions & options)
{
  gtsam::Vector sigmas(6);
  sigmas << options.bias_prediction_accelerometer_sigma,
    options.bias_prediction_accelerometer_sigma,
    options.bias_prediction_accelerometer_sigma,
    options.bias_prediction_gyroscope_sigma,
    options.bias_prediction_gyroscope_sigma,
    options.bias_prediction_gyroscope_sigma;
  return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

[[nodiscard]] gtsam::SharedNoiseModel make_bias_between_noise(
  const stereo_vio::VioBackendOptions & options,
  const double delta_time_seconds)
{
  const double sqrt_dt = std::sqrt(std::max(delta_time_seconds, 1.0e-6));
  gtsam::Vector sigmas(6);
  sigmas << options.accelerometer_bias_random_walk_sigma * sqrt_dt,
    options.accelerometer_bias_random_walk_sigma * sqrt_dt,
    options.accelerometer_bias_random_walk_sigma * sqrt_dt,
    options.gyroscope_bias_random_walk_sigma * sqrt_dt,
    options.gyroscope_bias_random_walk_sigma * sqrt_dt,
    options.gyroscope_bias_random_walk_sigma * sqrt_dt;
  return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

[[nodiscard]] gtsam::SharedNoiseModel make_visual_between_noise(
  const stereo_vio::VioBackendOptions & options)
{
  gtsam::Vector sigmas(6);
  sigmas << options.visual_between_rotation_sigma,
    options.visual_between_rotation_sigma,
    options.visual_between_rotation_sigma,
    options.visual_between_translation_sigma,
    options.visual_between_translation_sigma,
    options.visual_between_translation_sigma;
  return gtsam::noiseModel::Robust::Create(
    gtsam::noiseModel::mEstimator::Huber::Create(options.visual_huber_k),
    gtsam::noiseModel::Diagonal::Sigmas(sigmas));
}

[[nodiscard]] decltype(gtsam::PreintegrationParams::MakeSharedU(9.81))
make_preintegration_params(const stereo_vio::VioBackendOptions & options)
{
  decltype(gtsam::PreintegrationParams::MakeSharedU(9.81)) params =
    gtsam::PreintegrationParams::MakeSharedU(options.gravity_magnitude);
  params->accelerometerCovariance =
    gtsam::Matrix3::Identity() *
    options.accelerometer_noise_sigma *
    options.accelerometer_noise_sigma;
  params->gyroscopeCovariance =
    gtsam::Matrix3::Identity() *
    options.gyroscope_noise_sigma *
    options.gyroscope_noise_sigma;
  params->integrationCovariance =
    gtsam::Matrix3::Identity() *
    options.integration_noise_sigma *
    options.integration_noise_sigma;
  return params;
}

[[nodiscard]] bool integrate_imu_measurements(
  const double start_stamp_seconds,
  const double end_stamp_seconds,
  const std::vector<stereo_vio::VioImuSample> & imu_samples,
  gtsam::PreintegratedImuMeasurements & preintegrated,
  double & integrated_time_seconds)
{
  integrated_time_seconds = 0.0;
  if (end_stamp_seconds <= start_stamp_seconds || imu_samples.empty()) {
    return false;
  }

  double integration_stamp_seconds = start_stamp_seconds;
  bool has_integrated_sample = false;
  gtsam::Vector3 last_acceleration = gtsam::Vector3::Zero();
  gtsam::Vector3 last_angular_velocity = gtsam::Vector3::Zero();

  for (const stereo_vio::VioImuSample & imu_sample : imu_samples) {
    if (imu_sample.stamp_seconds <= start_stamp_seconds) {
      continue;
    }
    if (imu_sample.stamp_seconds > end_stamp_seconds) {
      break;
    }

    const double delta_time_seconds = imu_sample.stamp_seconds - integration_stamp_seconds;
    if (delta_time_seconds <= 0.0) {
      continue;
    }

    last_acceleration = make_vector3(imu_sample.linear_acceleration);
    last_angular_velocity = make_vector3(imu_sample.angular_velocity);
    preintegrated.integrateMeasurement(
      last_acceleration,
      last_angular_velocity,
      delta_time_seconds);
    integration_stamp_seconds = imu_sample.stamp_seconds;
    integrated_time_seconds += delta_time_seconds;
    has_integrated_sample = true;
  }

  if (!has_integrated_sample) {
    return false;
  }

  const double tail_delta_time_seconds = end_stamp_seconds - integration_stamp_seconds;
  if (tail_delta_time_seconds > 0.0) {
    preintegrated.integrateMeasurement(
      last_acceleration,
      last_angular_velocity,
      tail_delta_time_seconds);
    integrated_time_seconds += tail_delta_time_seconds;
  }

  return true;
}

[[nodiscard]] bool is_landmark_usable(
  const stereo_vio::VioFeatureObservation & observation,
  const int minimum_visual_track_age)
{
  return observation.triangulated &&
         observation.track_age >= minimum_visual_track_age &&
         observation.landmark_left.z > 0.0 &&
         std::isfinite(observation.landmark_left.x) &&
         std::isfinite(observation.landmark_left.y) &&
         std::isfinite(observation.landmark_left.z);
}

[[nodiscard]] RigidTransformEstimate estimate_rigid_transform(
  const std::vector<VisualCorrespondence> & correspondences,
  const std::vector<std::size_t> & indices)
{
  RigidTransformEstimate estimate;
  if (indices.size() < 3U) {
    return estimate;
  }

  Eigen::Vector3d previous_mean = Eigen::Vector3d::Zero();
  Eigen::Vector3d current_mean = Eigen::Vector3d::Zero();
  for (const std::size_t index : indices) {
    previous_mean += correspondences.at(index).previous_left_point;
    current_mean += correspondences.at(index).current_left_point;
  }
  previous_mean /= static_cast<double>(indices.size());
  current_mean /= static_cast<double>(indices.size());

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const std::size_t index : indices) {
    const Eigen::Vector3d previous_centered =
      correspondences.at(index).previous_left_point - previous_mean;
    const Eigen::Vector3d current_centered =
      correspondences.at(index).current_left_point - current_mean;
    covariance += current_centered * previous_centered.transpose();
  }

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
    covariance,
    Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d rotation = svd.matrixV() * svd.matrixU().transpose();
  if (rotation.determinant() < 0.0) {
    Eigen::Matrix3d matrix_v = svd.matrixV();
    matrix_v.col(2) *= -1.0;
    rotation = matrix_v * svd.matrixU().transpose();
  }

  estimate.previous_left_T_current_left = Eigen::Isometry3d::Identity();
  estimate.previous_left_T_current_left.linear() = rotation;
  estimate.previous_left_T_current_left.translation() =
    previous_mean - rotation * current_mean;

  double total_error = 0.0;
  for (const std::size_t index : indices) {
    const Eigen::Vector3d predicted_previous =
      estimate.previous_left_T_current_left *
      correspondences.at(index).current_left_point;
    total_error +=
      (correspondences.at(index).previous_left_point - predicted_previous).norm();
  }

  estimate.valid = true;
  estimate.inlier_indices = indices;
  estimate.mean_error = total_error / static_cast<double>(indices.size());
  return estimate;
}

[[nodiscard]] std::vector<std::size_t> find_rigid_transform_inliers(
  const std::vector<VisualCorrespondence> & correspondences,
  const Eigen::Isometry3d & previous_left_T_current_left,
  const double threshold)
{
  std::vector<std::size_t> inlier_indices;
  for (std::size_t index = 0U; index < correspondences.size(); ++index) {
    const Eigen::Vector3d predicted_previous =
      previous_left_T_current_left * correspondences.at(index).current_left_point;
    const double error =
      (correspondences.at(index).previous_left_point - predicted_previous).norm();
    if (error <= threshold) {
      inlier_indices.push_back(index);
    }
  }
  return inlier_indices;
}

[[nodiscard]] RigidTransformEstimate estimate_rigid_transform_ransac(
  const std::vector<VisualCorrespondence> & correspondences,
  const stereo_vio::VioBackendOptions & options)
{
  RigidTransformEstimate best_estimate;
  if (
    correspondences.size() <
    static_cast<std::size_t>(options.minimum_visual_odometry_inliers))
  {
    return best_estimate;
  }

  std::mt19937 rng(7U);
  std::uniform_int_distribution<std::size_t> index_distribution(
    0U,
    correspondences.size() - 1U);

  for (int iteration = 0; iteration < 64; ++iteration) {
    std::set<std::size_t> sample_set;
    while (sample_set.size() < 3U) {
      sample_set.insert(index_distribution(rng));
    }

    std::vector<std::size_t> sample_indices;
    sample_indices.reserve(sample_set.size());
    for (const std::size_t index : sample_set) {
      sample_indices.push_back(index);
    }

    const RigidTransformEstimate sample_estimate =
      estimate_rigid_transform(correspondences, sample_indices);
    if (!sample_estimate.valid) {
      continue;
    }

    const std::vector<std::size_t> inlier_indices =
      find_rigid_transform_inliers(
        correspondences,
        sample_estimate.previous_left_T_current_left,
        options.visual_3d_ransac_threshold);
    if (
      inlier_indices.size() < static_cast<std::size_t>(options.minimum_visual_odometry_inliers))
    {
      continue;
    }

    const RigidTransformEstimate refined_estimate =
      estimate_rigid_transform(correspondences, inlier_indices);
    if (!refined_estimate.valid) {
      continue;
    }

    if (
      !best_estimate.valid ||
      refined_estimate.inlier_indices.size() > best_estimate.inlier_indices.size() ||
      (refined_estimate.inlier_indices.size() == best_estimate.inlier_indices.size() &&
      refined_estimate.mean_error < best_estimate.mean_error))
    {
      best_estimate = refined_estimate;
    }
  }

  return best_estimate;
}

[[nodiscard]] VisualOdometryMeasurement estimate_visual_odometry(
  const std::vector<stereo_vio::VioFeatureObservation> & previous_observations,
  const std::vector<stereo_vio::VioFeatureObservation> & current_observations,
  const stereo_vio::VioBackendFrameTransforms & frame_transforms,
  const stereo_vio::VioBackendOptions & options)
{
  VisualOdometryMeasurement measurement;
  std::map<uint64_t, stereo_vio::VioFeatureObservation> previous_observation_by_id;
  for (const stereo_vio::VioFeatureObservation & previous_observation : previous_observations) {
    previous_observation_by_id.emplace(previous_observation.id, previous_observation);
  }

  std::vector<VisualCorrespondence> correspondences;
  correspondences.reserve(current_observations.size());
  for (const stereo_vio::VioFeatureObservation & current_observation : current_observations) {
    const std::map<uint64_t, stereo_vio::VioFeatureObservation>::const_iterator
      previous_iterator = previous_observation_by_id.find(current_observation.id);
    if (previous_iterator == previous_observation_by_id.end()) {
      continue;
    }

    VisualCorrespondence correspondence;
    correspondence.previous_left_point =
      make_vector3d(previous_iterator->second.landmark_left);
    correspondence.current_left_point = make_vector3d(current_observation.landmark_left);
    correspondences.push_back(correspondence);
  }

  const RigidTransformEstimate left_frame_estimate =
    estimate_rigid_transform_ransac(correspondences, options);
  if (!left_frame_estimate.valid) {
    return measurement;
  }

  const Eigen::Isometry3d previous_imu_T_current_imu =
    frame_transforms.imu_T_left_camera *
    left_frame_estimate.previous_left_T_current_left *
    frame_transforms.imu_T_left_camera.inverse();

  measurement.valid = true;
  measurement.previous_imu_T_current_imu = make_pose3(previous_imu_T_current_imu);
  return measurement;
}

}  // namespace

namespace stereo_vio
{

VioBackend::VioBackend()
: VioBackend(VioBackendOptions{})
{
}

VioBackend::VioBackend(const VioBackendOptions & options)
: options_(options),
  initialized_(false),
  next_frame_id_(0U),
  isam2_(std::make_unique<gtsam::ISAM2>(make_isam2_params(options)))
{
  if (!this->is_options_valid()) {
    throw std::invalid_argument("Invalid VIO backend options");
  }
}

VioBackendResult VioBackend::process_frame(
  const double stamp_seconds,
  const VioFrontendResult & frontend_result,
  const std::vector<VioImuSample> & imu_samples,
  const VioBackendFrameTransforms & frame_transforms)
{
  if (!this->initialized_) {
    std::vector<VioImuSample> initialization_imu_samples = imu_samples;
    if (this->options_.use_initial_gravity_alignment) {
      const double latest_initial_sample_stamp =
        this->initial_gravity_samples_.empty() ?
        -std::numeric_limits<double>::infinity() :
        this->initial_gravity_samples_.back().stamp_seconds;
      for (const VioImuSample & imu_sample : imu_samples) {
        if (imu_sample.stamp_seconds > latest_initial_sample_stamp) {
          this->initial_gravity_samples_.push_back(imu_sample);
        }
      }
      if (this->initial_gravity_samples_.size() > kMaxInitialGravityAlignmentSamples) {
        this->initial_gravity_samples_.erase(
          this->initial_gravity_samples_.begin(),
          this->initial_gravity_samples_.begin() +
          static_cast<std::vector<VioImuSample>::difference_type>(
            this->initial_gravity_samples_.size() - kMaxInitialGravityAlignmentSamples));
      }

      std::string gravity_alignment_status;
      if (
        !this->estimate_initial_odom_T_base_from_gravity(
          this->initial_gravity_samples_,
          frame_transforms,
          gravity_alignment_status).has_value())
      {
        VioBackendResult result;
        result.valid = false;
        result.initialized = false;
        result.status_message = gravity_alignment_status;
        return result;
      }

      initialization_imu_samples = this->initial_gravity_samples_;
    }

    BackendFrame frame = this->make_first_frame(
      stamp_seconds,
      frontend_result,
      initialization_imu_samples,
      frame_transforms);
    VioBackendResult result;
    if (!this->update_isam2(frame, frame_transforms, result)) {
      return result;
    }
    this->previous_frame_ = frame;
    this->initialized_ = true;
    this->initial_gravity_samples_.clear();
    result.initialized = true;
    ++this->next_frame_id_;
    return result;
  }

  if (stamp_seconds <= this->previous_frame_.stamp_seconds) {
    VioBackendResult result;
    result.valid = false;
    result.initialized = true;
    result.status_message = "received non-monotonic stereo timestamp";
    return result;
  }

  if (imu_samples.empty()) {
    VioBackendResult result;
    result.valid = false;
    result.initialized = true;
    result.status_message = "waiting for IMU samples up to the stereo timestamp";
    return result;
  }

  BackendFrame frame =
    this->make_next_frame(stamp_seconds, frontend_result, imu_samples);
  VioBackendResult result;
  if (!this->update_isam2(frame, frame_transforms, result)) {
    return result;
  }

  this->previous_frame_ = frame;
  ++this->next_frame_id_;
  return result;
}

bool VioBackend::is_options_valid() const
{
  return this->options_.gravity_magnitude > 0.0 &&
         this->options_.gravity_alignment_minimum_samples > 0 &&
         this->options_.gravity_alignment_acceleration_tolerance > 0.0 &&
         this->options_.gravity_alignment_max_angular_velocity > 0.0 &&
         this->options_.accelerometer_noise_sigma > 0.0 &&
         this->options_.gyroscope_noise_sigma > 0.0 &&
         this->options_.accelerometer_bias_random_walk_sigma > 0.0 &&
         this->options_.gyroscope_bias_random_walk_sigma > 0.0 &&
         this->options_.integration_noise_sigma > 0.0 &&
         this->options_.visual_between_rotation_sigma > 0.0 &&
         this->options_.visual_between_translation_sigma > 0.0 &&
         this->options_.visual_3d_ransac_threshold > 0.0 &&
         this->options_.visual_huber_k > 0.0 &&
         this->options_.pose_prior_rotation_sigma > 0.0 &&
         this->options_.pose_prior_translation_sigma > 0.0 &&
         this->options_.pose_prediction_rotation_sigma > 0.0 &&
         this->options_.pose_prediction_translation_sigma > 0.0 &&
         this->options_.velocity_prior_sigma > 0.0 &&
         this->options_.velocity_prediction_sigma > 0.0 &&
         this->options_.bias_prior_accelerometer_sigma > 0.0 &&
         this->options_.bias_prior_gyroscope_sigma > 0.0 &&
         this->options_.bias_prediction_accelerometer_sigma > 0.0 &&
         this->options_.bias_prediction_gyroscope_sigma > 0.0 &&
         this->options_.max_visual_observations_per_frame > 0 &&
         this->options_.minimum_visual_track_age >= 0 &&
         this->options_.minimum_visual_odometry_inliers >= 3 &&
         this->options_.isam2_relinearize_threshold > 0.0 &&
         this->options_.isam2_relinearize_skip > 0;
}

std::vector<VioFeatureObservation> VioBackend::make_backend_observations(
  const VioFrontendResult & frontend_result) const
{
  std::vector<VioFeatureObservation> observations;
  observations.reserve(
    std::min(
      frontend_result.observations.size(),
      static_cast<std::size_t>(this->options_.max_visual_observations_per_frame)));
  for (const VioFeatureObservation & observation : frontend_result.observations) {
    if (
      observations.size() >=
      static_cast<std::size_t>(this->options_.max_visual_observations_per_frame))
    {
      break;
    }

    if (is_landmark_usable(observation, this->options_.minimum_visual_track_age)) {
      observations.push_back(observation);
    }
  }
  return observations;
}

VioBackend::BackendFrame VioBackend::make_first_frame(
  const double stamp_seconds,
  const VioFrontendResult & frontend_result,
  const std::vector<VioImuSample> & imu_samples,
  const VioBackendFrameTransforms & frame_transforms) const
{
  BackendFrame frame;
  frame.id = this->next_frame_id_;
  frame.stamp_seconds = stamp_seconds;
  frame.observations = this->make_backend_observations(frontend_result);
  frame.imu_samples = imu_samples;

  // GTSAM's MakeSharedU assumes odom +Z is up, so level odom before the first IMU factor.
  Eigen::Isometry3d odom_T_base = Eigen::Isometry3d::Identity();
  if (this->options_.use_initial_gravity_alignment) {
    std::string gravity_alignment_status;
    const std::optional<Eigen::Isometry3d> gravity_aligned_odom_T_base =
      this->estimate_initial_odom_T_base_from_gravity(
        imu_samples,
        frame_transforms,
        gravity_alignment_status);
    if (gravity_aligned_odom_T_base.has_value()) {
      odom_T_base = gravity_aligned_odom_T_base.value();
    }
    frame.status_message = gravity_alignment_status;
  }

  frame.odom_T_imu = make_pose3(odom_T_base * frame_transforms.imu_T_base.inverse());
  frame.velocity_odom = gtsam::Vector3::Zero();
  frame.imu_bias = gtsam::imuBias::ConstantBias();
  if (frame.status_message.empty()) {
    frame.status_message = "isam2 initialized";
  }
  return frame;
}

VioBackend::BackendFrame VioBackend::make_next_frame(
  const double stamp_seconds,
  const VioFrontendResult & frontend_result,
  const std::vector<VioImuSample> & imu_samples) const
{
  BackendFrame frame;
  frame.id = this->next_frame_id_;
  frame.stamp_seconds = stamp_seconds;
  frame.observations = this->make_backend_observations(frontend_result);
  frame.imu_samples = imu_samples;
  frame.odom_T_imu = this->previous_frame_.odom_T_imu;
  frame.velocity_odom = this->previous_frame_.velocity_odom;
  frame.imu_bias = this->previous_frame_.imu_bias;

  decltype(gtsam::PreintegrationParams::MakeSharedU(9.81)) preintegration_params =
    make_preintegration_params(this->options_);
  gtsam::PreintegratedImuMeasurements preintegrated(
    preintegration_params,
    this->previous_frame_.imu_bias);
  double integrated_time_seconds = 0.0;
  const bool has_imu =
    integrate_imu_measurements(
      this->previous_frame_.stamp_seconds,
      stamp_seconds,
      imu_samples,
      preintegrated,
      integrated_time_seconds);
  if (has_imu) {
    const gtsam::NavState previous_state(
      this->previous_frame_.odom_T_imu,
      this->previous_frame_.velocity_odom);
    const gtsam::NavState predicted_state =
      preintegrated.predict(previous_state, this->previous_frame_.imu_bias);
    frame.odom_T_imu = predicted_state.pose();
    frame.velocity_odom = predicted_state.v();
  }

  return frame;
}

VioBackendResult VioBackend::make_result(
  const BackendFrame & frame,
  const VioBackendFrameTransforms & frame_transforms,
  const std::string & status_message) const
{
  const gtsam::Pose3 imu_T_base = make_pose3(frame_transforms.imu_T_base);
  const gtsam::Pose3 odom_T_base = frame.odom_T_imu.compose(imu_T_base);
  Eigen::Isometry3d result_odom_T_base = make_isometry3d(odom_T_base);
  Eigen::Vector3d result_velocity_odom(
    frame.velocity_odom.x(),
    frame.velocity_odom.y(),
    frame.velocity_odom.z());

  if (this->options_.use_planar_3dof_mode) {
    result_odom_T_base = make_planar_3dof_transform(result_odom_T_base);
    result_velocity_odom.z() = 0.0;
  }

  VioBackendResult result;
  result.valid = true;
  result.initialized = this->initialized_;
  result.frame_id = frame.id;
  result.stamp_seconds = frame.stamp_seconds;
  result.odom_T_base = result_odom_T_base;
  result.velocity_odom = result_velocity_odom;
  result.status_message = status_message;
  return result;
}

bool VioBackend::update_isam2(
  BackendFrame & frame,
  const VioBackendFrameTransforms & frame_transforms,
  VioBackendResult & result)
{
  result.initialized = this->initialized_;

  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;

  new_values.insert(make_pose_key(frame.id), frame.odom_T_imu);
  new_values.insert(make_velocity_key(frame.id), frame.velocity_odom);
  new_values.insert(make_bias_key(frame.id), frame.imu_bias);

  if (!this->initialized_) {
    new_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
      make_pose_key(frame.id),
      frame.odom_T_imu,
      make_pose_prior_noise(this->options_)));
    new_factors.add(gtsam::PriorFactor<gtsam::Vector3>(
      make_velocity_key(frame.id),
      frame.velocity_odom,
      make_velocity_prior_noise(this->options_)));
    new_factors.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
      make_bias_key(frame.id),
      frame.imu_bias,
      make_bias_prior_noise(this->options_)));
  } else {
    decltype(gtsam::PreintegrationParams::MakeSharedU(9.81)) preintegration_params =
      make_preintegration_params(this->options_);
    gtsam::PreintegratedImuMeasurements preintegrated(
      preintegration_params,
      this->previous_frame_.imu_bias);
    double integrated_time_seconds = 0.0;
    const bool has_imu =
      integrate_imu_measurements(
        this->previous_frame_.stamp_seconds,
        frame.stamp_seconds,
        frame.imu_samples,
        preintegrated,
        integrated_time_seconds);
    if (!has_imu) {
      result.valid = false;
      result.status_message = "waiting for IMU samples between stereo frames";
      return false;
    }

    new_factors.add(gtsam::ImuFactor(
      make_pose_key(this->previous_frame_.id),
      make_velocity_key(this->previous_frame_.id),
      make_pose_key(frame.id),
      make_velocity_key(frame.id),
      make_bias_key(this->previous_frame_.id),
      preintegrated));
    new_factors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
      make_bias_key(this->previous_frame_.id),
      make_bias_key(frame.id),
      gtsam::imuBias::ConstantBias(),
      make_bias_between_noise(this->options_, integrated_time_seconds)));

    new_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
      make_pose_key(frame.id),
      frame.odom_T_imu,
      make_pose_prediction_noise(this->options_)));
    new_factors.add(gtsam::PriorFactor<gtsam::Vector3>(
      make_velocity_key(frame.id),
      frame.velocity_odom,
      make_velocity_prediction_noise(this->options_)));
    new_factors.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
      make_bias_key(frame.id),
      frame.imu_bias,
      make_bias_prediction_noise(this->options_)));

    const VisualOdometryMeasurement visual_odometry =
      estimate_visual_odometry(
        this->previous_frame_.observations,
        frame.observations,
        frame_transforms,
        this->options_);
    if (visual_odometry.valid) {
      new_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
        make_pose_key(this->previous_frame_.id),
        make_pose_key(frame.id),
        visual_odometry.previous_imu_T_current_imu,
        make_visual_between_noise(this->options_)));
    }
  }

  try {
    this->isam2_->update(new_factors, new_values);
    const gtsam::Values estimate = this->isam2_->calculateEstimate();
    frame.odom_T_imu = estimate.at<gtsam::Pose3>(make_pose_key(frame.id));
    frame.velocity_odom = estimate.at<gtsam::Vector3>(make_velocity_key(frame.id));
    frame.imu_bias =
      estimate.at<gtsam::imuBias::ConstantBias>(make_bias_key(frame.id));
  } catch (const std::exception & exception) {
    return this->reset_isam2_at_frame(
      frame,
      frame_transforms,
      std::string("isam2 restarted at predicted odom: ") + exception.what(),
      result);
  }

  const std::string update_status =
    this->initialized_ ? "isam2 updated" : frame.status_message;
  result = this->make_result(
    frame,
    frame_transforms,
    update_status);
  return true;
}

bool VioBackend::reset_isam2_at_frame(
  const BackendFrame & frame,
  const VioBackendFrameTransforms & frame_transforms,
  const std::string & status_message,
  VioBackendResult & result)
{
  try {
    this->isam2_ = std::make_unique<gtsam::ISAM2>(make_isam2_params(this->options_));

    gtsam::NonlinearFactorGraph reset_factors;
    gtsam::Values reset_values;
    reset_values.insert(make_pose_key(frame.id), frame.odom_T_imu);
    reset_values.insert(make_velocity_key(frame.id), frame.velocity_odom);
    reset_values.insert(make_bias_key(frame.id), frame.imu_bias);
    reset_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
      make_pose_key(frame.id),
      frame.odom_T_imu,
      make_pose_prior_noise(this->options_)));
    reset_factors.add(gtsam::PriorFactor<gtsam::Vector3>(
      make_velocity_key(frame.id),
      frame.velocity_odom,
      make_velocity_prior_noise(this->options_)));
    reset_factors.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
      make_bias_key(frame.id),
      frame.imu_bias,
      make_bias_prior_noise(this->options_)));

    this->isam2_->update(reset_factors, reset_values);
  } catch (const std::exception & exception) {
    result.valid = false;
    result.status_message = exception.what();
    return false;
  }

  result = this->make_result(frame, frame_transforms, status_message);
  return true;
}

std::optional<Eigen::Isometry3d> VioBackend::estimate_initial_odom_T_base_from_gravity(
  const std::vector<VioImuSample> & imu_samples,
  const VioBackendFrameTransforms & frame_transforms,
  std::string & status_message) const
{
  if (
    imu_samples.size() <
    static_cast<std::size_t>(this->options_.gravity_alignment_minimum_samples))
  {
    status_message =
      "isam2 initialized without gravity alignment: not enough IMU samples (" +
      std::to_string(imu_samples.size()) + "/" +
      std::to_string(this->options_.gravity_alignment_minimum_samples) + ")";
    return std::nullopt;
  }

  Eigen::Vector3d mean_acceleration_imu = Eigen::Vector3d::Zero();
  double mean_angular_velocity_norm = 0.0;
  for (const VioImuSample & imu_sample : imu_samples) {
    mean_acceleration_imu += imu_sample.linear_acceleration;
    mean_angular_velocity_norm += imu_sample.angular_velocity.norm();
  }
  mean_acceleration_imu /= static_cast<double>(imu_samples.size());
  mean_angular_velocity_norm /= static_cast<double>(imu_samples.size());

  const double acceleration_norm = mean_acceleration_imu.norm();
  if (!std::isfinite(acceleration_norm) || acceleration_norm <= 0.0) {
    status_message =
      "isam2 initialized without gravity alignment: invalid mean acceleration";
    return std::nullopt;
  }

  if (
    std::fabs(acceleration_norm - this->options_.gravity_magnitude) >
    this->options_.gravity_alignment_acceleration_tolerance)
  {
    status_message =
      "isam2 initialized without gravity alignment: acceleration is not gravity-like";
    return std::nullopt;
  }

  if (mean_angular_velocity_norm > this->options_.gravity_alignment_max_angular_velocity) {
    status_message =
      "isam2 initialized without gravity alignment: IMU is rotating too much";
    return std::nullopt;
  }

  const Eigen::Isometry3d nominal_base_T_imu = frame_transforms.imu_T_base.inverse();
  const Eigen::Vector3d measured_up_base =
    nominal_base_T_imu.linear() * mean_acceleration_imu.normalized();
  if (!measured_up_base.allFinite() || measured_up_base.norm() <= 0.0) {
    status_message =
      "isam2 initialized without gravity alignment: invalid measured up vector";
    return std::nullopt;
  }

  Eigen::Quaterniond odom_q_base;
  odom_q_base.setFromTwoVectors(measured_up_base, Eigen::Vector3d::UnitZ());
  odom_q_base.normalize();

  Eigen::Isometry3d odom_T_base = Eigen::Isometry3d::Identity();
  odom_T_base.linear() = odom_q_base.toRotationMatrix();
  status_message = "isam2 initialized with gravity-aligned odom frame";
  return odom_T_base;
}

}  // namespace stereo_vio
