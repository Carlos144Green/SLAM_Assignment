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

#include "stereo_vio/stereo_calibration_publisher.hpp"

#include "rclcpp_components/register_node_macro.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

constexpr char kLeftCameraInfoTopic[] = "left/camera_info";
constexpr char kRightCameraInfoTopic[] = "right/camera_info";
constexpr char kLeftRectifiedCameraInfoTopic[] = "left/rectified_camera_info";
constexpr char kRightRectifiedCameraInfoTopic[] = "right/rectified_camera_info";
constexpr char kLeftImageTopic[] = "left/image";
constexpr char kRightImageTopic[] = "right/image";
constexpr char kDistortionModel[] = "plumb_bob";
constexpr char kBaseFrameId[] = "base_link";
constexpr std::size_t kCalibrationQueueDepth = 1000;

constexpr std::array<double, 9> kIdentityRectification{
  1.0, 0.0, 0.0,
  0.0, 1.0, 0.0,
  0.0, 0.0, 1.0};

template<typename T, std::size_t Size>
[[nodiscard]] std::array<T, Size> required_array(
  rclcpp::Node & node,
  const std::string & parameter_name)
{
  const std::vector<T> values =
    node.declare_parameter<std::vector<T>>(parameter_name, std::vector<T>{});
  if (values.size() != Size) {
    throw std::invalid_argument(
      "Required parameter '" + parameter_name + "' must contain exactly " +
      std::to_string(Size) + " values");
  }

  std::array<T, Size> fixed_values{};
  std::copy(values.begin(), values.end(), fixed_values.begin());
  return fixed_values;
}

[[nodiscard]] std::string required_string(rclcpp::Node & node, const std::string & parameter_name)
{
  std::string value = node.declare_parameter<std::string>(parameter_name, "");
  if (value.empty()) {
    throw std::invalid_argument("Required parameter '" + parameter_name + "' cannot be empty");
  }
  return value;
}

[[nodiscard]] std::string camera_frame_id_from_optical_frame_id(
  const std::string & optical_frame_id)
{
  const std::string suffix = "_optical_frame";
  if (
    optical_frame_id.size() >= suffix.size() &&
    optical_frame_id.compare(optical_frame_id.size() - suffix.size(), suffix.size(), suffix) == 0)
  {
    return optical_frame_id.substr(0, optical_frame_id.size() - suffix.size()) + "_camera_frame";
  }
  return optical_frame_id + "_camera_frame";
}

[[nodiscard]] Eigen::Vector4d required_vector4d(
  rclcpp::Node & node,
  const std::string & parameter_name)
{
  const std::array<double, 4> values = required_array<double, 4>(node, parameter_name);
  return Eigen::Vector4d(values[0], values[1], values[2], values[3]);
}

[[nodiscard]] Eigen::Matrix4d required_matrix4d(
  rclcpp::Node & node,
  const std::string & parameter_name)
{
  const std::array<double, 16> values = required_array<double, 16>(node, parameter_name);
  Eigen::Matrix4d transform;
  transform << values[0], values[1], values[2], values[3],
    values[4], values[5], values[6], values[7],
    values[8], values[9], values[10], values[11],
    values[12], values[13], values[14], values[15];
  return transform;
}

[[nodiscard]] Eigen::Matrix4d inverse_rigid_transform(
  const Eigen::Matrix4d & transform)
{
  Eigen::Matrix4d inverse_transform = Eigen::Matrix4d::Identity();
  const Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
  const Eigen::Vector3d translation = transform.block<3, 1>(0, 3);
  inverse_transform.block<3, 3>(0, 0) = rotation.transpose();
  inverse_transform.block<3, 1>(0, 3) = -rotation.transpose() * translation;
  return inverse_transform;
}

[[nodiscard]] Eigen::Matrix3d optical_from_body_rotation()
{
  Eigen::Matrix3d rotation;
  rotation << 0.0, -1.0, 0.0,
    0.0, 0.0, -1.0,
    1.0, 0.0, 0.0;
  return rotation;
}

[[nodiscard]] Eigen::Matrix4d optical_from_camera_transform()
{
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) = optical_from_body_rotation();
  return transform;
}

[[nodiscard]] Eigen::Matrix3d rotation_matrix_from_transform(
  const Eigen::Matrix4d & transform)
{
  return transform.block<3, 3>(0, 0);
}

[[nodiscard]] Eigen::Vector3d translation_vector_from_transform(
  const Eigen::Matrix4d & transform)
{
  return transform.block<3, 1>(0, 3);
}

[[nodiscard]] Eigen::Matrix3d camera_matrix_from_intrinsics(
  const Eigen::Vector4d & intrinsics)
{
  Eigen::Matrix3d camera_matrix = Eigen::Matrix3d::Identity();
  camera_matrix(0, 0) = intrinsics(0);
  camera_matrix(0, 2) = intrinsics(2);
  camera_matrix(1, 1) = intrinsics(1);
  camera_matrix(1, 2) = intrinsics(3);
  return camera_matrix;
}

[[nodiscard]] geometry_msgs::msg::Quaternion eigen_quaternion_to_msg(
  const Eigen::Quaterniond & quaternion)
{
  Eigen::Quaterniond normalized_quaternion = quaternion.normalized();
  geometry_msgs::msg::Quaternion message;
  message.x = normalized_quaternion.x();
  message.y = normalized_quaternion.y();
  message.z = normalized_quaternion.z();
  message.w = normalized_quaternion.w();
  return message;
}

[[nodiscard]] rclcpp::QoS make_calibration_qos()
{
  rclcpp::QoS qos{kCalibrationQueueDepth};
  qos.reliable();
  qos.durability_volatile();
  return qos;
}

}  // namespace

namespace stereo_vio
{

StereoCalibrationPublisher::StereoCalibrationPublisher(const rclcpp::NodeOptions & options)
: rclcpp::Node("stereo_calibration_publisher", options),
  imu_frame_id_(required_string(*this, "imu_frame_id")),
  image_size_(required_array<int64_t, 2>(*this, "image_size")),
  left_camera_(this->load_camera("left")),
  right_camera_(this->load_camera("right")),
  stereo_center_in_imu_(this->compute_stereo_center_in_imu()),
  base_rotation_in_imu_(this->compute_base_rotation_in_imu())
{
  this->compute_rectified_camera_infos();

  const rclcpp::QoS calibration_qos = make_calibration_qos();

  this->left_camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
    kLeftCameraInfoTopic,
    calibration_qos);
  this->right_camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
    kRightCameraInfoTopic,
    calibration_qos);
  this->left_rectified_camera_info_pub_ =
    this->create_publisher<sensor_msgs::msg::CameraInfo>(
      kLeftRectifiedCameraInfoTopic,
      calibration_qos);
  this->right_rectified_camera_info_pub_ =
    this->create_publisher<sensor_msgs::msg::CameraInfo>(
      kRightRectifiedCameraInfoTopic,
      calibration_qos);

  this->left_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    kLeftImageTopic,
    calibration_qos,
    std::bind(
      &StereoCalibrationPublisher::publish_left_camera_info,
      this,
      std::placeholders::_1));
  this->right_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    kRightImageTopic,
    calibration_qos,
    std::bind(
      &StereoCalibrationPublisher::publish_right_camera_info,
      this,
      std::placeholders::_1));

  this->frame_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
  this->publish_camera_frames();
}

StereoCalibrationPublisher::CameraCalibration StereoCalibrationPublisher::load_camera(
  const std::string & name)
{
  const std::string optical_frame_id = required_string(*this, name + ".frame_id");
  const Eigen::Matrix4d t_imu_optical_frame = required_matrix4d(*this, name + ".T_imu_camera");
  return {
    camera_frame_id_from_optical_frame_id(optical_frame_id),
    optical_frame_id,
    required_vector4d(*this, name + ".intrinsics"),
    required_vector4d(*this, name + ".distortion"),
    t_imu_optical_frame * optical_from_camera_transform(),
    t_imu_optical_frame,
    inverse_rigid_transform(t_imu_optical_frame)};
}

sensor_msgs::msg::CameraInfo StereoCalibrationPublisher::make_camera_info(
  const CameraCalibration & camera) const
{
  const double fx = camera.intrinsics[0];
  const double fy = camera.intrinsics[1];
  const double cx = camera.intrinsics[2];
  const double cy = camera.intrinsics[3];

  sensor_msgs::msg::CameraInfo camera_info;
  camera_info.header.frame_id = camera.optical_frame_id;
  camera_info.width = static_cast<uint32_t>(this->image_size_[0]);
  camera_info.height = static_cast<uint32_t>(this->image_size_[1]);
  camera_info.distortion_model = kDistortionModel;
  camera_info.d = {
    camera.distortion(0),
    camera.distortion(1),
    camera.distortion(2),
    camera.distortion(3),
    0.0};
  camera_info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  camera_info.r = kIdentityRectification;
  camera_info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  return camera_info;
}

sensor_msgs::msg::CameraInfo StereoCalibrationPublisher::make_rectified_camera_info(
  const sensor_msgs::msg::CameraInfo & camera_info) const
{
  sensor_msgs::msg::CameraInfo rectified_camera_info = camera_info;
  rectified_camera_info.distortion_model = kDistortionModel;
  rectified_camera_info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  rectified_camera_info.k = {
    camera_info.p[0], camera_info.p[1], camera_info.p[2],
    camera_info.p[4], camera_info.p[5], camera_info.p[6],
    camera_info.p[8], camera_info.p[9], camera_info.p[10]};
  rectified_camera_info.r = kIdentityRectification;
  return rectified_camera_info;
}

void StereoCalibrationPublisher::compute_rectified_camera_infos()
{
  if (this->image_size_[0] <= 0 || this->image_size_[1] <= 0) {
    throw std::invalid_argument("Required parameter 'image_size' must contain positive values");
  }

  this->left_camera_info_ = this->make_camera_info(this->left_camera_);
  this->right_camera_info_ = this->make_camera_info(this->right_camera_);

  const Eigen::Matrix3d r_left_imu =
    rotation_matrix_from_transform(this->left_camera_.t_optical_frame_imu);
  const Eigen::Vector3d t_left_imu =
    translation_vector_from_transform(this->left_camera_.t_optical_frame_imu);
  const Eigen::Matrix3d r_right_imu =
    rotation_matrix_from_transform(this->right_camera_.t_optical_frame_imu);
  const Eigen::Vector3d t_right_imu =
    translation_vector_from_transform(this->right_camera_.t_optical_frame_imu);

  const Eigen::Matrix3d r_left_right = r_right_imu * r_left_imu.transpose();
  const Eigen::Vector3d t_left_right = t_right_imu - r_left_right * t_left_imu;

  const Eigen::Matrix3d k_left_eigen =
    camera_matrix_from_intrinsics(this->left_camera_.intrinsics);
  const Eigen::Matrix3d k_right_eigen =
    camera_matrix_from_intrinsics(this->right_camera_.intrinsics);
  const Eigen::Matrix<double, 1, 4> d_left_eigen = this->left_camera_.distortion.transpose();
  const Eigen::Matrix<double, 1, 4> d_right_eigen = this->right_camera_.distortion.transpose();

  cv::Mat k_left;
  cv::Mat k_right;
  cv::Mat d_left;
  cv::Mat d_right;
  cv::Mat r_left_right_cv;
  cv::Mat t_left_right_cv;
  cv::eigen2cv(k_left_eigen, k_left);
  cv::eigen2cv(k_right_eigen, k_right);
  cv::eigen2cv(d_left_eigen, d_left);
  cv::eigen2cv(d_right_eigen, d_right);
  cv::eigen2cv(r_left_right, r_left_right_cv);
  cv::eigen2cv(t_left_right, t_left_right_cv);

  cv::Mat r1;
  cv::Mat r2;
  cv::Mat p1;
  cv::Mat p2;
  cv::Mat q;
  const cv::Size image_size(
    static_cast<int>(this->image_size_[0]),
    static_cast<int>(this->image_size_[1]));

  cv::stereoRectify(
    k_left,
    d_left,
    k_right,
    d_right,
    image_size,
    r_left_right_cv,
    t_left_right_cv,
    r1,
    r2,
    p1,
    p2,
    q);

  for (int index = 0; index < 9; ++index) {
    this->left_camera_info_.r[static_cast<std::size_t>(index)] =
      r1.at<double>(index / 3, index % 3);
    this->right_camera_info_.r[static_cast<std::size_t>(index)] =
      r2.at<double>(index / 3, index % 3);
  }

  for (int index = 0; index < 12; ++index) {
    this->left_camera_info_.p[static_cast<std::size_t>(index)] =
      p1.at<double>(index / 4, index % 4);
    this->right_camera_info_.p[static_cast<std::size_t>(index)] =
      p2.at<double>(index / 4, index % 4);
  }

  this->left_rectified_camera_info_ =
    this->make_rectified_camera_info(this->left_camera_info_);
  this->right_rectified_camera_info_ =
    this->make_rectified_camera_info(this->right_camera_info_);
}

Eigen::Vector3d StereoCalibrationPublisher::compute_stereo_center_in_imu() const
{
  return 0.5 * (
    translation_vector_from_transform(this->left_camera_.t_imu_camera_frame) +
    translation_vector_from_transform(this->right_camera_.t_imu_camera_frame));
}

Eigen::Matrix3d StereoCalibrationPublisher::compute_base_rotation_in_imu() const
{
  const Eigen::Matrix3d left_camera_rotation_in_imu =
    rotation_matrix_from_transform(this->left_camera_.t_imu_camera_frame);
  Eigen::Quaterniond base_rotation_quaternion(left_camera_rotation_in_imu);
  base_rotation_quaternion.normalize();
  return base_rotation_quaternion.toRotationMatrix();
}

Eigen::Matrix4d StereoCalibrationPublisher::make_imu_to_base_transform() const
{
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) = this->base_rotation_in_imu_;
  transform.block<3, 1>(0, 3) = this->stereo_center_in_imu_;
  return transform;
}

Eigen::Matrix4d StereoCalibrationPublisher::make_base_to_imu_transform() const
{
  return inverse_rigid_transform(this->make_imu_to_base_transform());
}

Eigen::Matrix4d StereoCalibrationPublisher::make_base_to_camera_transform(
  const CameraCalibration & camera) const
{
  return this->make_base_to_imu_transform() * camera.t_imu_camera_frame;
}

Eigen::Matrix4d StereoCalibrationPublisher::make_camera_to_optical_transform() const
{
  return inverse_rigid_transform(optical_from_camera_transform());
}

geometry_msgs::msg::TransformStamped StereoCalibrationPublisher::make_transform(
  const std::string & parent_frame_id,
  const std::string & child_frame_id,
  const Eigen::Matrix4d & transform_matrix,
  const rclcpp::Time & stamp) const
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = parent_frame_id;
  transform.child_frame_id = child_frame_id;
  transform.transform.translation.x = transform_matrix(0, 3);
  transform.transform.translation.y = transform_matrix(1, 3);
  transform.transform.translation.z = transform_matrix(2, 3);
  transform.transform.rotation = eigen_quaternion_to_msg(
    Eigen::Quaterniond(rotation_matrix_from_transform(transform_matrix)));
  return transform;
}

void StereoCalibrationPublisher::publish_left_camera_info(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg)
{
  this->left_camera_info_.header.stamp = image_msg->header.stamp;
  this->left_rectified_camera_info_.header.stamp = image_msg->header.stamp;
  this->left_camera_info_pub_->publish(this->left_camera_info_);
  this->left_rectified_camera_info_pub_->publish(this->left_rectified_camera_info_);
}

void StereoCalibrationPublisher::publish_right_camera_info(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg)
{
  this->right_camera_info_.header.stamp = image_msg->header.stamp;
  this->right_rectified_camera_info_.header.stamp = image_msg->header.stamp;
  this->right_camera_info_pub_->publish(this->right_camera_info_);
  this->right_rectified_camera_info_pub_->publish(this->right_rectified_camera_info_);
}

void StereoCalibrationPublisher::publish_camera_frames()
{
  const rclcpp::Time stamp = this->now();
  const Eigen::Matrix4d base_to_imu_transform = this->make_base_to_imu_transform();
  const Eigen::Matrix4d base_to_left_camera_transform =
    this->make_base_to_camera_transform(this->left_camera_);
  const Eigen::Matrix4d base_to_right_camera_transform =
    this->make_base_to_camera_transform(this->right_camera_);
  const Eigen::Matrix4d camera_to_optical_transform =
    this->make_camera_to_optical_transform();

  this->frame_broadcaster_->sendTransform(
    std::vector<geometry_msgs::msg::TransformStamped>{
      this->make_transform(kBaseFrameId, this->imu_frame_id_, base_to_imu_transform, stamp),
      this->make_transform(
        kBaseFrameId,
        this->left_camera_.camera_frame_id,
        base_to_left_camera_transform,
        stamp),
      this->make_transform(
        kBaseFrameId,
        this->right_camera_.camera_frame_id,
        base_to_right_camera_transform,
        stamp),
      this->make_transform(
        this->left_camera_.camera_frame_id,
        this->left_camera_.optical_frame_id,
        camera_to_optical_transform,
        stamp),
      this->make_transform(
        this->right_camera_.camera_frame_id,
        this->right_camera_.optical_frame_id,
        camera_to_optical_transform,
        stamp)});
}

}  // namespace stereo_vio

RCLCPP_COMPONENTS_REGISTER_NODE(stereo_vio::StereoCalibrationPublisher)
