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

#include "stereo_vio/vio_frontend.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace
{

[[nodiscard]] cv::Mat make_gray_image(const cv::Mat & image)
{
  if (image.empty()) {
    throw std::invalid_argument("Frontend image cannot be empty");
  }

  cv::Mat gray_image;
  if (image.channels() == 1) {
    gray_image = image;
  } else if (image.channels() == 3) {
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
  } else if (image.channels() == 4) {
    cv::cvtColor(image, gray_image, cv::COLOR_BGRA2GRAY);
  } else {
    throw std::invalid_argument("Unsupported image channel count for frontend grayscale input");
  }

  if (gray_image.type() != CV_8UC1) {
    cv::Mat converted_gray_image;
    gray_image.convertTo(converted_gray_image, CV_8UC1);
    return converted_gray_image;
  }

  return gray_image;
}

[[nodiscard]] cv::Point make_draw_point(const cv::Point2f & point)
{
  return cv::Point(
    static_cast<int>(std::lround(point.x)),
    static_cast<int>(std::lround(point.y)));
}

}  // namespace

namespace stereo_vio
{

VioFrontend::VioFrontend()
: VioFrontend(Options{})
{
}

VioFrontend::VioFrontend(const Options & options)
: options_(options),
  next_feature_id_(1U)
{
  if (!this->is_options_valid()) {
    throw std::invalid_argument("Invalid VIO frontend options");
  }
}

VioFrontendResult VioFrontend::process_stereo(
  const cv::Mat & left_image,
  const cv::Mat & right_image,
  const VioCameraModel & camera_model)
{
  if (
    camera_model.fx <= 0.0 || camera_model.fy <= 0.0 ||
    camera_model.baseline <= 0.0)
  {
    throw std::invalid_argument("Invalid rectified stereo camera model");
  }

  const cv::Mat left_gray = make_gray_image(left_image);
  const cv::Mat right_gray = make_gray_image(right_image);

  std::vector<cv::Point2f> left_points;
  std::vector<uint64_t> left_feature_ids;
  std::vector<int> left_track_ages;
  std::vector<cv::Point2f> right_points;
  std::vector<uint64_t> right_feature_ids;
  std::vector<int> right_track_ages;

  this->track_features(
    this->previous_left_gray_,
    left_gray,
    this->previous_left_points_,
    this->previous_left_feature_ids_,
    this->previous_left_track_ages_,
    left_points,
    left_feature_ids,
    left_track_ages);
  this->track_features(
    this->previous_right_gray_,
    right_gray,
    this->previous_right_points_,
    this->previous_right_feature_ids_,
    this->previous_right_track_ages_,
    right_points,
    right_feature_ids,
    right_track_ages);

  this->add_new_features(left_gray, left_points, left_feature_ids, left_track_ages);
  this->add_new_features(right_gray, right_points, right_feature_ids, right_track_ages);

  VioFrontendResult result;
  result.observations = this->match_stereo_and_triangulate(
    left_gray,
    right_gray,
    left_points,
    left_feature_ids,
    left_track_ages,
    camera_model);

  this->update_previous_state(
    left_gray,
    left_points,
    left_feature_ids,
    left_track_ages,
    this->previous_left_gray_,
    this->previous_left_points_,
    this->previous_left_feature_ids_,
    this->previous_left_track_ages_);
  this->update_previous_state(
    right_gray,
    right_points,
    right_feature_ids,
    right_track_ages,
    this->previous_right_gray_,
    this->previous_right_points_,
    this->previous_right_feature_ids_,
    this->previous_right_track_ages_);
  return result;
}

bool VioFrontend::is_options_valid() const
{
  return this->options_.max_features > 0 &&
         this->options_.min_tracked_features > 0 &&
         this->options_.min_tracked_features <= this->options_.max_features &&
         this->options_.block_size > 0 &&
         this->options_.lk_window_size > 2 &&
         this->options_.lk_max_level >= 0 &&
         this->options_.quality_level > 0.0 &&
         this->options_.quality_level < 1.0 &&
         this->options_.min_feature_distance > 0.0 &&
         this->options_.max_lk_error > 0.0 &&
         this->options_.max_forward_backward_error >= 0.0 &&
         this->options_.max_stereo_y_error >= 0.0 &&
         this->options_.stereo_ransac_threshold > 0.0 &&
         this->options_.stereo_ransac_confidence > 0.0 &&
         this->options_.stereo_ransac_confidence < 1.0 &&
         this->options_.min_disparity > 0.0 &&
         this->options_.min_depth > 0.0 &&
         this->options_.max_depth > this->options_.min_depth &&
         this->options_.pixel_mask_border >= 0;
}

bool VioFrontend::is_point_in_feature_area(
  const cv::Point2f & point,
  const cv::Size & size) const
{
  const float border = static_cast<float>(this->options_.pixel_mask_border);
  return point.x >= border &&
         point.y >= border &&
         point.x < static_cast<float>(size.width) - border &&
         point.y < static_cast<float>(size.height) - border;
}

bool VioFrontend::is_forward_backward_consistent(
  const cv::Point2f & original_point,
  const cv::Point2f & backward_point) const
{
  const double dx = static_cast<double>(original_point.x - backward_point.x);
  const double dy = static_cast<double>(original_point.y - backward_point.y);
  const double error = std::sqrt(dx * dx + dy * dy);
  return error <= this->options_.max_forward_backward_error;
}

cv::Mat VioFrontend::make_feature_mask(
  const cv::Size & image_size,
  const std::vector<cv::Point2f> & existing_points) const
{
  cv::Mat mask(image_size, CV_8UC1, cv::Scalar(0));
  const int border = this->options_.pixel_mask_border;
  if (image_size.width <= 2 * border || image_size.height <= 2 * border) {
    return mask;
  }

  const cv::Rect valid_region(
    border,
    border,
    image_size.width - 2 * border,
    image_size.height - 2 * border);
  mask(valid_region).setTo(cv::Scalar(255));

  const int exclusion_radius =
    std::max(1, static_cast<int>(std::lround(this->options_.min_feature_distance)));
  for (const cv::Point2f & point : existing_points) {
    cv::circle(mask, make_draw_point(point), exclusion_radius, cv::Scalar(0), -1);
  }

  return mask;
}

void VioFrontend::track_features(
  const cv::Mat & previous_gray,
  const cv::Mat & current_gray,
  const std::vector<cv::Point2f> & previous_points,
  const std::vector<uint64_t> & previous_feature_ids,
  const std::vector<int> & previous_track_ages,
  std::vector<cv::Point2f> & current_points,
  std::vector<uint64_t> & feature_ids,
  std::vector<int> & track_ages) const
{
  if (previous_gray.empty() || previous_points.empty()) {
    return;
  }

  std::vector<cv::Point2f> tracked_points;
  std::vector<unsigned char> status;
  std::vector<float> errors;
  const cv::TermCriteria criteria(
    cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
    30,
    0.01);

  cv::calcOpticalFlowPyrLK(
    previous_gray,
    current_gray,
    previous_points,
    tracked_points,
    status,
    errors,
    cv::Size(this->options_.lk_window_size, this->options_.lk_window_size),
    this->options_.lk_max_level,
    criteria);

  std::vector<cv::Point2f> backward_points;
  std::vector<unsigned char> backward_status;
  std::vector<float> backward_errors;
  cv::calcOpticalFlowPyrLK(
    current_gray,
    previous_gray,
    tracked_points,
    backward_points,
    backward_status,
    backward_errors,
    cv::Size(this->options_.lk_window_size, this->options_.lk_window_size),
    this->options_.lk_max_level,
    criteria);

  const cv::Size image_size = current_gray.size();
  for (std::size_t index = 0U; index < tracked_points.size(); ++index) {
    if (
      status[index] == 0U ||
      backward_status[index] == 0U ||
      errors[index] > static_cast<float>(this->options_.max_lk_error) ||
      backward_errors[index] > static_cast<float>(this->options_.max_lk_error) ||
      !this->is_point_in_feature_area(tracked_points[index], image_size) ||
      !this->is_forward_backward_consistent(previous_points[index], backward_points[index]))
    {
      continue;
    }

    current_points.push_back(tracked_points[index]);
    feature_ids.push_back(previous_feature_ids[index]);
    track_ages.push_back(previous_track_ages[index] + 1);
  }
}

void VioFrontend::add_new_features(
  const cv::Mat & gray_image,
  std::vector<cv::Point2f> & points,
  std::vector<uint64_t> & feature_ids,
  std::vector<int> & track_ages)
{
  if (points.size() >= static_cast<std::size_t>(this->options_.max_features)) {
    return;
  }

  if (points.size() >= static_cast<std::size_t>(this->options_.min_tracked_features)) {
    return;
  }

  const cv::Mat mask = this->make_feature_mask(gray_image.size(), points);

  const std::size_t open_feature_count =
    static_cast<std::size_t>(this->options_.max_features) - points.size();
  const int features_to_add = static_cast<int>(open_feature_count);

  std::vector<cv::Point2f> new_points;
  cv::goodFeaturesToTrack(
    gray_image,
    new_points,
    features_to_add,
    this->options_.quality_level,
    this->options_.min_feature_distance,
    mask,
    this->options_.block_size);

  if (!new_points.empty()) {
    const cv::TermCriteria criteria(
      cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
      20,
      0.03);
    cv::cornerSubPix(
      gray_image,
      new_points,
      cv::Size(3, 3),
      cv::Size(-1, -1),
      criteria);
  }

  for (const cv::Point2f & point : new_points) {
    if (!this->is_point_in_feature_area(point, gray_image.size())) {
      continue;
    }
    points.push_back(point);
    feature_ids.push_back(this->next_feature_id_);
    track_ages.push_back(1);
    ++this->next_feature_id_;
  }
}

std::vector<VioFeatureObservation> VioFrontend::match_stereo_and_triangulate(
  const cv::Mat & left_gray,
  const cv::Mat & right_gray,
  const std::vector<cv::Point2f> & left_points,
  const std::vector<uint64_t> & feature_ids,
  const std::vector<int> & track_ages,
  const VioCameraModel & camera_model) const
{
  std::vector<VioFeatureObservation> observations;
  observations.reserve(left_points.size());

  if (left_points.empty()) {
    return observations;
  }

  std::vector<cv::Point2f> right_points;
  std::vector<unsigned char> status;
  std::vector<float> errors;
  const cv::TermCriteria criteria(
    cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
    30,
    0.01);

  cv::calcOpticalFlowPyrLK(
    left_gray,
    right_gray,
    left_points,
    right_points,
    status,
    errors,
    cv::Size(this->options_.lk_window_size, this->options_.lk_window_size),
    this->options_.lk_max_level,
    criteria);

  std::vector<cv::Point2f> backward_points;
  std::vector<unsigned char> backward_status;
  std::vector<float> backward_errors;
  cv::calcOpticalFlowPyrLK(
    right_gray,
    left_gray,
    right_points,
    backward_points,
    backward_status,
    backward_errors,
    cv::Size(this->options_.lk_window_size, this->options_.lk_window_size),
    this->options_.lk_max_level,
    criteria);

  std::vector<unsigned char> candidate_mask(left_points.size(), 0U);
  const cv::Size right_size = right_gray.size();
  for (std::size_t index = 0U; index < left_points.size(); ++index) {
    if (
      status[index] != 0U &&
      backward_status[index] != 0U &&
      errors[index] <= static_cast<float>(this->options_.max_lk_error) &&
      backward_errors[index] <= static_cast<float>(this->options_.max_lk_error) &&
      this->is_point_in_feature_area(left_points[index], left_gray.size()) &&
      this->is_point_in_feature_area(right_points[index], right_size) &&
      this->is_forward_backward_consistent(left_points[index], backward_points[index]))
    {
      candidate_mask[index] = 1U;
    }
  }

  const std::vector<unsigned char> ransac_inlier_mask =
    this->make_stereo_ransac_inlier_mask(left_points, right_points, candidate_mask);

  for (std::size_t index = 0U; index < left_points.size(); ++index) {
    VioFeatureObservation observation;
    observation.id = feature_ids[index];
    observation.left_pixel = left_points[index];
    observation.right_pixel = right_points[index];
    observation.landmark_left = cv::Point3d(0.0, 0.0, 0.0);
    observation.track_age = track_ages[index];
    observation.triangulated = false;

    if (candidate_mask[index] != 0U && ransac_inlier_mask[index] != 0U) {
      const double vertical_error =
        std::abs(static_cast<double>(left_points[index].y - right_points[index].y));
      const double disparity =
        static_cast<double>(left_points[index].x - right_points[index].x);

      if (
        vertical_error <= this->options_.max_stereo_y_error &&
        disparity >= this->options_.min_disparity)
      {
        const double depth = camera_model.fx * camera_model.baseline / disparity;
        if (depth >= this->options_.min_depth && depth <= this->options_.max_depth) {
          observation.landmark_left.x =
            (static_cast<double>(left_points[index].x) - camera_model.cx) *
            depth / camera_model.fx;
          observation.landmark_left.y =
            (static_cast<double>(left_points[index].y) - camera_model.cy) *
            depth / camera_model.fy;
          observation.landmark_left.z = depth;
          observation.triangulated = true;
        }
      }
    }

    observations.push_back(observation);
  }

  return observations;
}

std::vector<unsigned char> VioFrontend::make_stereo_ransac_inlier_mask(
  const std::vector<cv::Point2f> & left_points,
  const std::vector<cv::Point2f> & right_points,
  const std::vector<unsigned char> & candidate_mask) const
{
  std::vector<unsigned char> inlier_mask(left_points.size(), 0U);
  std::vector<cv::Point2f> candidate_left_points;
  std::vector<cv::Point2f> candidate_right_points;
  std::vector<std::size_t> candidate_indices;
  candidate_left_points.reserve(left_points.size());
  candidate_right_points.reserve(left_points.size());
  candidate_indices.reserve(left_points.size());

  for (std::size_t index = 0U; index < candidate_mask.size(); ++index) {
    if (candidate_mask[index] == 0U) {
      continue;
    }

    candidate_left_points.push_back(left_points[index]);
    candidate_right_points.push_back(right_points[index]);
    candidate_indices.push_back(index);
  }

  if (candidate_left_points.size() < 8U) {
    for (const std::size_t index : candidate_indices) {
      inlier_mask[index] = 1U;
    }
    return inlier_mask;
  }

  std::vector<unsigned char> compact_inlier_mask;
  cv::findFundamentalMat(
    candidate_left_points,
    candidate_right_points,
    cv::FM_RANSAC,
    this->options_.stereo_ransac_threshold,
    this->options_.stereo_ransac_confidence,
    compact_inlier_mask);

  if (compact_inlier_mask.size() != candidate_indices.size()) {
    return inlier_mask;
  }

  for (std::size_t index = 0U; index < compact_inlier_mask.size(); ++index) {
    if (compact_inlier_mask[index] != 0U) {
      inlier_mask[candidate_indices[index]] = 1U;
    }
  }

  return inlier_mask;
}

void VioFrontend::update_previous_state(
  const cv::Mat & gray_image,
  const std::vector<cv::Point2f> & points,
  const std::vector<uint64_t> & feature_ids,
  const std::vector<int> & track_ages,
  cv::Mat & previous_gray,
  std::vector<cv::Point2f> & previous_points,
  std::vector<uint64_t> & previous_feature_ids,
  std::vector<int> & previous_track_ages)
{
  previous_gray = gray_image.clone();
  previous_points = points;
  previous_feature_ids = feature_ids;
  previous_track_ages = track_ages;
}

}  // namespace stereo_vio
