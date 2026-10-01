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

#include <rerun.hpp>

#include <cstdint>
#include <vector>

namespace rerun_wrapper
{

[[nodiscard]] std::vector<uint8_t> encode_jpeg(const sensor_msgs::msg::Image & msg);
[[nodiscard]] int64_t stamp_to_ns(const builtin_interfaces::msg::Time & stamp);
[[nodiscard]] rerun::Pinhole pinhole_from_camera_info(
  const sensor_msgs::msg::CameraInfo & camera_info);
[[nodiscard]] rerun::Transform3D transform_from_odom(const nav_msgs::msg::Odometry & odom);
[[nodiscard]] rerun::Transform3D transform_from_tf(
  const geometry_msgs::msg::TransformStamped & transform);
[[nodiscard]] rerun::datatypes::Vec3D trajectory_point_from_odom(
  const nav_msgs::msg::Odometry & odom);
[[nodiscard]] float squared_distance(
  const rerun::datatypes::Vec3D & first,
  const rerun::datatypes::Vec3D & second);

}  // namespace rerun_wrapper
