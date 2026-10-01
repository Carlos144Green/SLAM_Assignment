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

#include "rerun_wrapper/rerun_conversions.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <array>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace rerun_wrapper
{

namespace
{

constexpr int kJpegQuality = 80;

[[nodiscard]] bool has_rectified_projection(const sensor_msgs::msg::CameraInfo & camera_info)
{
  for (const double value : camera_info.p) {
    if (value != 0.0) {
      return true;
    }
  }
  return false;
}

[[nodiscard]] rerun::Transform3D transform_from_translation_rotation(
  const double translation_x,
  const double translation_y,
  const double translation_z,
  const geometry_msgs::msg::Quaternion & orientation)
{
  return rerun::Transform3D::from_translation_rotation(
    rerun::components::Translation3D{
      static_cast<float>(translation_x),
      static_cast<float>(translation_y),
      static_cast<float>(translation_z)},
    rerun::datatypes::Quaternion::from_xyzw(
      static_cast<float>(orientation.x),
      static_cast<float>(orientation.y),
      static_cast<float>(orientation.z),
      static_cast<float>(orientation.w)));
}

}  // namespace

std::vector<uint8_t> encode_jpeg(const sensor_msgs::msg::Image & msg)
{
  const cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, msg.encoding);
  const cv::Mat & image = cv_ptr->image;
  cv::Mat image_to_encode;

  if (msg.encoding == "bgr8") {
    image_to_encode = image;
  } else if (msg.encoding == "rgb8") {
    cv::cvtColor(image, image_to_encode, cv::COLOR_RGB2BGR);
  } else if (msg.encoding == "mono8") {
    image_to_encode = image;
  } else {
    throw std::invalid_argument("Unsupported image encoding for Rerun logging: " + msg.encoding);
  }

  std::vector<uint8_t> encoded;
  const std::vector<int> encode_params{cv::IMWRITE_JPEG_QUALITY, kJpegQuality};
  if (!cv::imencode(".jpg", image_to_encode, encoded, encode_params)) {
    throw std::runtime_error("Failed to JPEG-encode image for Rerun logging");
  }
  return encoded;
}

int64_t stamp_to_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + static_cast<int64_t>(stamp.nanosec);
}

rerun::Pinhole pinhole_from_camera_info(const sensor_msgs::msg::CameraInfo & camera_info)
{
  std::array<float, 9> image_from_camera{};

  if (has_rectified_projection(camera_info)) {
    image_from_camera = {
      static_cast<float>(camera_info.p[0]),
      static_cast<float>(camera_info.p[4]),
      static_cast<float>(camera_info.p[8]),
      static_cast<float>(camera_info.p[1]),
      static_cast<float>(camera_info.p[5]),
      static_cast<float>(camera_info.p[9]),
      static_cast<float>(camera_info.p[2]),
      static_cast<float>(camera_info.p[6]),
      static_cast<float>(camera_info.p[10])};
  } else {
    image_from_camera = {
      static_cast<float>(camera_info.k[0]),
      static_cast<float>(camera_info.k[3]),
      static_cast<float>(camera_info.k[6]),
      static_cast<float>(camera_info.k[1]),
      static_cast<float>(camera_info.k[4]),
      static_cast<float>(camera_info.k[7]),
      static_cast<float>(camera_info.k[2]),
      static_cast<float>(camera_info.k[5]),
      static_cast<float>(camera_info.k[8])};
  }

  return rerun::Pinhole(rerun::components::PinholeProjection{image_from_camera})
    .with_resolution(static_cast<int>(camera_info.width), static_cast<int>(camera_info.height));
}

rerun::Transform3D transform_from_odom(const nav_msgs::msg::Odometry & odom)
{
  const geometry_msgs::msg::Point & position = odom.pose.pose.position;
  const geometry_msgs::msg::Quaternion & orientation = odom.pose.pose.orientation;

  return transform_from_translation_rotation(position.x, position.y, position.z, orientation);
}

rerun::Transform3D transform_from_tf(const geometry_msgs::msg::TransformStamped & transform)
{
  const geometry_msgs::msg::Vector3 & translation = transform.transform.translation;
  const geometry_msgs::msg::Quaternion & orientation = transform.transform.rotation;
  return transform_from_translation_rotation(
    translation.x,
    translation.y,
    translation.z,
    orientation);
}

rerun::datatypes::Vec3D trajectory_point_from_odom(const nav_msgs::msg::Odometry & odom)
{
  const geometry_msgs::msg::Point & position = odom.pose.pose.position;
  return rerun::datatypes::Vec3D{
    static_cast<float>(position.x),
    static_cast<float>(position.y),
    static_cast<float>(position.z)};
}

float squared_distance(
  const rerun::datatypes::Vec3D & first,
  const rerun::datatypes::Vec3D & second)
{
  const float dx = first.x() - second.x();
  const float dy = first.y() - second.y();
  const float dz = first.z() - second.z();
  return dx * dx + dy * dy + dz * dz;
}

}  // namespace rerun_wrapper
