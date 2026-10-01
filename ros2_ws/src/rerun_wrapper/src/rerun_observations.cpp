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

#include "rerun_wrapper/rerun_observations.hpp"

#include <geometry_msgs/msg/point32.hpp>

#include <algorithm>
#include <cstddef>
#include <vector>

namespace rerun_wrapper
{

std::size_t stereo_observation_count(
  const stereo_vio_msgs::msg::StereoObservations & observations)
{
  const std::size_t left_count =
    std::min(observations.left_u.size(), observations.left_v.size());
  const std::size_t right_count =
    std::min(observations.right_u.size(), observations.right_v.size());
  const std::size_t pixel_count = std::min(left_count, right_count);
  const std::size_t geometry_count = std::min(pixel_count, observations.landmarks_left.size());
  return std::min(geometry_count, observations.feature_ids.size());
}

std::vector<rerun::Position2D> make_left_observation_points(
  const stereo_vio_msgs::msg::StereoObservations & observations)
{
  const std::size_t observation_count = stereo_observation_count(observations);
  std::vector<rerun::Position2D> points;
  points.reserve(observation_count);

  for (std::size_t index = 0U; index < observation_count; ++index) {
    points.emplace_back(observations.left_u[index], observations.left_v[index]);
  }

  return points;
}

std::vector<rerun::Position2D> make_right_observation_points(
  const stereo_vio_msgs::msg::StereoObservations & observations)
{
  const std::size_t observation_count = stereo_observation_count(observations);
  std::vector<rerun::Position2D> points;
  points.reserve(observation_count);

  for (std::size_t index = 0U; index < observation_count; ++index) {
    points.emplace_back(observations.right_u[index], observations.right_v[index]);
  }

  return points;
}

std::vector<rerun::Position3D> make_landmark_points_left(
  const stereo_vio_msgs::msg::StereoObservations & observations)
{
  const std::size_t observation_count = stereo_observation_count(observations);
  std::vector<rerun::Position3D> points;
  points.reserve(observation_count);

  for (std::size_t index = 0U; index < observation_count; ++index) {
    const geometry_msgs::msg::Point32 & landmark = observations.landmarks_left[index];
    points.emplace_back(landmark.x, landmark.y, landmark.z);
  }

  return points;
}

}  // namespace rerun_wrapper
