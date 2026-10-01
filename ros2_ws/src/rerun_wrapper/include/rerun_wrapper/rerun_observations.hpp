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

#include "stereo_vio_msgs/msg/stereo_observations.hpp"

#include <rerun.hpp>

#include <cstddef>
#include <vector>

namespace rerun_wrapper
{

[[nodiscard]] std::size_t stereo_observation_count(
  const stereo_vio_msgs::msg::StereoObservations & observations);
[[nodiscard]] std::vector<rerun::Position2D> make_left_observation_points(
  const stereo_vio_msgs::msg::StereoObservations & observations);
[[nodiscard]] std::vector<rerun::Position2D> make_right_observation_points(
  const stereo_vio_msgs::msg::StereoObservations & observations);
[[nodiscard]] std::vector<rerun::Position3D> make_landmark_points_left(
  const stereo_vio_msgs::msg::StereoObservations & observations);

}  // namespace rerun_wrapper
