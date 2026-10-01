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

#include <opencv2/core.hpp>

#include <cstdint>
#include <vector>

namespace stereo_vio
{

struct VioCameraModel
{
  double fx;
  double fy;
  double cx;
  double cy;
  double baseline;
};

struct VioFeatureObservation
{
  uint64_t id;
  cv::Point2f left_pixel;
  cv::Point2f right_pixel;
  cv::Point3d landmark_left;
  int track_age;
  bool triangulated;
};

struct VioFrontendResult
{
  std::vector<VioFeatureObservation> observations;
};

class VioFrontend final
{
public:
  struct Options
  {
    int max_features = 500;
    int min_tracked_features = 250;
    int block_size = 7;
    int lk_window_size = 21;
    int lk_max_level = 3;
    double quality_level = 0.03;
    double min_feature_distance = 16.0;
    double max_lk_error = 20.0;
    double max_forward_backward_error = 1.0;
    double max_stereo_y_error = 2.0;
    double stereo_ransac_threshold = 1.0;
    double stereo_ransac_confidence = 0.995;
    double min_disparity = 1.0;
    double min_depth = 0.1;
    double max_depth = 80.0;
    int pixel_mask_border = 20;
  };

  VioFrontend();
  explicit VioFrontend(const Options & options);
  ~VioFrontend() = default;

  VioFrontend(const VioFrontend &) = delete;
  VioFrontend & operator=(const VioFrontend &) = delete;
  VioFrontend(VioFrontend &&) = delete;
  VioFrontend & operator=(VioFrontend &&) = delete;

  [[nodiscard]] VioFrontendResult process_stereo(
    const cv::Mat & left_image,
    const cv::Mat & right_image,
    const VioCameraModel & camera_model);

private:
  [[nodiscard]] bool is_options_valid() const;
  [[nodiscard]] bool is_point_in_feature_area(
    const cv::Point2f & point,
    const cv::Size & size) const;
  [[nodiscard]] bool is_forward_backward_consistent(
    const cv::Point2f & original_point,
    const cv::Point2f & backward_point) const;
  [[nodiscard]] cv::Mat make_feature_mask(
    const cv::Size & image_size,
    const std::vector<cv::Point2f> & existing_points) const;
  [[nodiscard]] std::vector<VioFeatureObservation> match_stereo_and_triangulate(
    const cv::Mat & left_gray,
    const cv::Mat & right_gray,
    const std::vector<cv::Point2f> & left_points,
    const std::vector<uint64_t> & feature_ids,
    const std::vector<int> & track_ages,
    const VioCameraModel & camera_model) const;
  [[nodiscard]] std::vector<unsigned char> make_stereo_ransac_inlier_mask(
    const std::vector<cv::Point2f> & left_points,
    const std::vector<cv::Point2f> & right_points,
    const std::vector<unsigned char> & candidate_mask) const;

  void track_features(
    const cv::Mat & previous_gray,
    const cv::Mat & current_gray,
    const std::vector<cv::Point2f> & previous_points,
    const std::vector<uint64_t> & previous_feature_ids,
    const std::vector<int> & previous_track_ages,
    std::vector<cv::Point2f> & current_points,
    std::vector<uint64_t> & feature_ids,
    std::vector<int> & track_ages) const;
  void add_new_features(
    const cv::Mat & gray_image,
    std::vector<cv::Point2f> & points,
    std::vector<uint64_t> & feature_ids,
    std::vector<int> & track_ages);
  void update_previous_state(
    const cv::Mat & gray_image,
    const std::vector<cv::Point2f> & points,
    const std::vector<uint64_t> & feature_ids,
    const std::vector<int> & track_ages,
    cv::Mat & previous_gray,
    std::vector<cv::Point2f> & previous_points,
    std::vector<uint64_t> & previous_feature_ids,
    std::vector<int> & previous_track_ages);

  Options options_;
  uint64_t next_feature_id_;
  cv::Mat previous_left_gray_;
  cv::Mat previous_right_gray_;
  std::vector<cv::Point2f> previous_left_points_;
  std::vector<cv::Point2f> previous_right_points_;
  std::vector<uint64_t> previous_left_feature_ids_;
  std::vector<uint64_t> previous_right_feature_ids_;
  std::vector<int> previous_left_track_ages_;
  std::vector<int> previous_right_track_ages_;
};

}  // namespace stereo_vio
