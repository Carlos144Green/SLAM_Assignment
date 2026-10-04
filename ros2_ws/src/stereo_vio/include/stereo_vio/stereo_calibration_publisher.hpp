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

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <string>

namespace stereo_vio
{

class StereoCalibrationPublisher final : public rclcpp::Node
{
public:
  explicit StereoCalibrationPublisher(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~StereoCalibrationPublisher() override = default;

  StereoCalibrationPublisher(const StereoCalibrationPublisher &) = delete;
  StereoCalibrationPublisher & operator=(const StereoCalibrationPublisher &) = delete;
  StereoCalibrationPublisher(StereoCalibrationPublisher &&) = delete;
  StereoCalibrationPublisher & operator=(StereoCalibrationPublisher &&) = delete;

private:
  struct CameraCalibration
  {
    std::string camera_frame_id;
    std::string optical_frame_id;
    Eigen::Vector4d intrinsics;
    double xi;
    Eigen::Vector4d distortion;
    Eigen::Matrix4d t_imu_camera_frame;
    Eigen::Matrix4d t_imu_optical_frame;
    Eigen::Matrix4d t_optical_frame_imu;
  };

  [[nodiscard]] CameraCalibration load_camera(const std::string & name);
  [[nodiscard]] sensor_msgs::msg::CameraInfo make_camera_info(
    const CameraCalibration & camera) const;
  [[nodiscard]] sensor_msgs::msg::CameraInfo make_rectified_camera_info(
    const sensor_msgs::msg::CameraInfo & camera_info) const;
  [[nodiscard]] Eigen::Vector3d compute_stereo_center_in_imu() const;
  [[nodiscard]] Eigen::Matrix3d compute_base_rotation_in_imu() const;
  [[nodiscard]] Eigen::Matrix4d make_imu_to_base_transform() const;
  [[nodiscard]] Eigen::Matrix4d make_base_to_imu_transform() const;
  [[nodiscard]] Eigen::Matrix4d make_base_to_camera_transform(
    const CameraCalibration & camera) const;
  [[nodiscard]] Eigen::Matrix4d make_camera_to_optical_transform() const;
  [[nodiscard]] geometry_msgs::msg::TransformStamped make_transform(
    const std::string & parent_frame_id,
    const std::string & child_frame_id,
    const Eigen::Matrix4d & transform_matrix,
    const rclcpp::Time & stamp) const;

  void compute_rectified_camera_infos();
  [[nodiscard]] sensor_msgs::msg::Image rectify_image(
    const sensor_msgs::msg::Image & image_msg,
    const cv::Mat & map_x,
    const cv::Mat & map_y) const;
  void publish_left_camera_info(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);
  void publish_right_camera_info(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);
  void publish_camera_frames();

  std::string imu_frame_id_;
  std::array<int64_t, 2> image_size_;
  CameraCalibration left_camera_;
  CameraCalibration right_camera_;
  Eigen::Vector3d stereo_center_in_imu_;
  Eigen::Matrix3d base_rotation_in_imu_;
  sensor_msgs::msg::CameraInfo left_camera_info_;
  sensor_msgs::msg::CameraInfo right_camera_info_;
  sensor_msgs::msg::CameraInfo left_rectified_camera_info_;
  sensor_msgs::msg::CameraInfo right_rectified_camera_info_;
  cv::Mat left_map_x_;
  cv::Mat left_map_y_;
  cv::Mat right_map_x_;
  cv::Mat right_map_y_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_camera_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_camera_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_rectified_camera_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_rectified_camera_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_rect_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_rect_image_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_image_sub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> frame_broadcaster_;
};

}  // namespace stereo_vio
