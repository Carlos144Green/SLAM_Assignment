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
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
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
constexpr char kLeftRectImageTopic[] = "left/image_rect";
constexpr char kRightRectImageTopic[] = "right/image_rect";
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

[[nodiscard]] Eigen::Matrix3d perspective_matrix_from_omni(
  const Eigen::Vector4d & intrinsics,
  const double xi)
{
  if (xi <= -1.0) {
    throw std::invalid_argument("Omni xi must be greater than -1");
  }
  Eigen::Matrix3d camera_matrix = camera_matrix_from_intrinsics(intrinsics);
  const double central_scale = 1.0 + xi;
  camera_matrix(0, 0) /= central_scale;
  camera_matrix(1, 1) /= central_scale;
  return camera_matrix;
}

[[nodiscard]] bool project_mei(
  const Eigen::Vector3d & point,
  const double xi,
  const Eigen::Vector4d & intrinsics,
  const Eigen::Vector4d & distortion,
  float & pixel_u,
  float & pixel_v)
{
  const double depth_norm = point.norm();
  const double fov_parameter = xi <= 1.0 ? xi : 1.0 / xi;
  if (point.z() <= -(fov_parameter * depth_norm)) {
    return false;
  }

  const double rz = 1.0 / (point.z() + xi * depth_norm);
  double x = point.x() * rz;
  double y = point.y() * rz;
  const double x2 = x * x;
  const double y2 = y * y;
  const double xy = x * y;
  const double r2 = x2 + y2;
  const double radial = distortion(0) * r2 + distortion(1) * r2 * r2;
  x += x * radial + 2.0 * distortion(2) * xy + distortion(3) * (r2 + 2.0 * x2);
  y += y * radial + 2.0 * distortion(3) * xy + distortion(2) * (r2 + 2.0 * y2);
  pixel_u = static_cast<float>(intrinsics(0) * x + intrinsics(2));
  pixel_v = static_cast<float>(intrinsics(1) * y + intrinsics(3));
  return std::isfinite(pixel_u) && std::isfinite(pixel_v);
}

void build_omni_rectify_maps(
  const Eigen::Vector4d & intrinsics,
  const double xi,
  const Eigen::Vector4d & distortion,
  const cv::Mat & rectification,
  const cv::Mat & projection,
  const cv::Size & image_size,
  cv::Mat & map_x,
  cv::Mat & map_y)
{
  Eigen::Matrix3d rectification_eigen;
  cv::cv2eigen(rectification, rectification_eigen);
  const Eigen::Matrix3d inverse_rectification = rectification_eigen.transpose();
  const double fx = projection.at<double>(0, 0);
  const double fy = projection.at<double>(1, 1);
  const double cx = projection.at<double>(0, 2);
  const double cy = projection.at<double>(1, 2);
  if (fx == 0.0 || fy == 0.0) {
    throw std::invalid_argument("Rectified projection focal length cannot be zero");
  }

  map_x.create(image_size, CV_32FC1);
  map_y.create(image_size, CV_32FC1);
  for (int row = 0; row < image_size.height; ++row) {
    float * map_x_row = map_x.ptr<float>(row);
    float * map_y_row = map_y.ptr<float>(row);
    for (int col = 0; col < image_size.width; ++col) {
      const Eigen::Vector3d rectified_ray(
        (static_cast<double>(col) - cx) / fx,
        (static_cast<double>(row) - cy) / fy,
        1.0);
      const Eigen::Vector3d camera_ray = inverse_rectification * rectified_ray;
      float pixel_u = -1.0F;
      float pixel_v = -1.0F;
      if (!project_mei(camera_ray, xi, intrinsics, distortion, pixel_u, pixel_v)) {
        pixel_u = -1.0F;
        pixel_v = -1.0F;
      }
      map_x_row[col] = pixel_u;
      map_y_row[col] = pixel_v;
    }
  }
}

[[nodiscard]] int image_type_from_encoding(
  const std::string & encoding,
  std::size_t & bytes_per_pixel)
{
  if (encoding == "mono8" || encoding == "8UC1") {
    bytes_per_pixel = 1U;
    return CV_8UC1;
  }
  if (encoding == "bgr8" || encoding == "rgb8" || encoding == "8UC3") {
    bytes_per_pixel = 3U;
    return CV_8UC3;
  }
  if (encoding == "bgra8" || encoding == "rgba8" || encoding == "8UC4") {
    bytes_per_pixel = 4U;
    return CV_8UC4;
  }
  throw std::invalid_argument("Unsupported image encoding: " + encoding);
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
  this->left_rect_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    kLeftRectImageTopic,
    calibration_qos);
  this->right_rect_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    kRightRectImageTopic,
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
    this->declare_parameter<double>(name + ".xi", 0.0),
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
  camera_info.d = camera.xi == 0.0 ?
    std::vector<double>{
      camera.distortion(0),
      camera.distortion(1),
      camera.distortion(2),
      camera.distortion(3),
      0.0} :
    std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0};
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

  const bool use_omni_model =
    this->left_camera_.xi != 0.0 || this->right_camera_.xi != 0.0;
  const Eigen::Matrix3d k_left_eigen = use_omni_model ?
    perspective_matrix_from_omni(this->left_camera_.intrinsics, this->left_camera_.xi) :
    camera_matrix_from_intrinsics(this->left_camera_.intrinsics);
  const Eigen::Matrix3d k_right_eigen = use_omni_model ?
    perspective_matrix_from_omni(this->right_camera_.intrinsics, this->right_camera_.xi) :
    camera_matrix_from_intrinsics(this->right_camera_.intrinsics);
  const Eigen::Matrix<double, 1, 4> d_left_eigen = use_omni_model ?
    Eigen::Matrix<double, 1, 4>::Zero() :
    this->left_camera_.distortion.transpose();
  const Eigen::Matrix<double, 1, 4> d_right_eigen = use_omni_model ?
    Eigen::Matrix<double, 1, 4>::Zero() :
    this->right_camera_.distortion.transpose();

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

  if (use_omni_model) {
    build_omni_rectify_maps(
      this->left_camera_.intrinsics,
      this->left_camera_.xi,
      this->left_camera_.distortion,
      r1,
      p1,
      image_size,
      this->left_map_x_,
      this->left_map_y_);
    build_omni_rectify_maps(
      this->right_camera_.intrinsics,
      this->right_camera_.xi,
      this->right_camera_.distortion,
      r2,
      p2,
      image_size,
      this->right_map_x_,
      this->right_map_y_);
    return;
  }

  cv::initUndistortRectifyMap(
    k_left,
    d_left,
    r1,
    p1,
    image_size,
    CV_32FC1,
    this->left_map_x_,
    this->left_map_y_);
  cv::initUndistortRectifyMap(
    k_right,
    d_right,
    r2,
    p2,
    image_size,
    CV_32FC1,
    this->right_map_x_,
    this->right_map_y_);
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

sensor_msgs::msg::Image StereoCalibrationPublisher::rectify_image(
  const sensor_msgs::msg::Image & image_msg,
  const cv::Mat & map_x,
  const cv::Mat & map_y) const
{
  if (image_msg.width == 0U || image_msg.height == 0U) {
    throw std::invalid_argument("Image message dimensions cannot be zero");
  }
  if (
    static_cast<int>(image_msg.width) != map_x.cols ||
    static_cast<int>(image_msg.height) != map_x.rows)
  {
    throw std::invalid_argument("Image message size does not match the calibration image size");
  }

  std::size_t bytes_per_pixel = 0U;
  const int image_type = image_type_from_encoding(image_msg.encoding, bytes_per_pixel);
  const std::size_t minimum_step = static_cast<std::size_t>(image_msg.width) * bytes_per_pixel;
  if (image_msg.step < minimum_step) {
    throw std::invalid_argument("Image message step is smaller than image width");
  }
  const std::size_t required_bytes =
    static_cast<std::size_t>(image_msg.step) *
    static_cast<std::size_t>(image_msg.height - 1U) + minimum_step;
  if (image_msg.data.size() < required_bytes) {
    throw std::invalid_argument("Image message data is smaller than expected");
  }

  const cv::Mat raw_image(
    static_cast<int>(image_msg.height),
    static_cast<int>(image_msg.width),
    image_type,
    const_cast<unsigned char *>(image_msg.data.data()),
    image_msg.step);
  cv::Mat rectified_image;
  cv::remap(
    raw_image,
    rectified_image,
    map_x,
    map_y,
    cv::INTER_CUBIC,
    cv::BORDER_CONSTANT);

  sensor_msgs::msg::Image rectified_message;
  rectified_message.header = image_msg.header;
  rectified_message.height = static_cast<uint32_t>(rectified_image.rows);
  rectified_message.width = static_cast<uint32_t>(rectified_image.cols);
  rectified_message.encoding = image_msg.encoding;
  rectified_message.is_bigendian = image_msg.is_bigendian;
  rectified_message.step =
    static_cast<uint32_t>(rectified_image.cols) *
    static_cast<uint32_t>(rectified_image.elemSize());
  const std::size_t rectified_bytes =
    static_cast<std::size_t>(rectified_message.step) *
    static_cast<std::size_t>(rectified_message.height);
  rectified_message.data.resize(rectified_bytes);
  if (rectified_image.isContinuous() &&
    rectified_image.step == static_cast<size_t>(rectified_message.step))
  {
    const unsigned char * source = rectified_image.ptr<unsigned char>(0);
    std::copy(source, source + rectified_bytes, rectified_message.data.begin());
  } else {
    for (int row = 0; row < rectified_image.rows; ++row) {
      const unsigned char * source_row = rectified_image.ptr<unsigned char>(row);
      unsigned char * destination_row =
        rectified_message.data.data() +
        static_cast<std::size_t>(row) * rectified_message.step;
      std::copy(source_row, source_row + rectified_message.step, destination_row);
    }
  }
  return rectified_message;
}

void StereoCalibrationPublisher::publish_left_camera_info(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg)
{
  this->left_camera_info_.header.stamp = image_msg->header.stamp;
  this->left_rectified_camera_info_.header.stamp = image_msg->header.stamp;
  this->left_camera_info_pub_->publish(this->left_camera_info_);
  this->left_rectified_camera_info_pub_->publish(this->left_rectified_camera_info_);
  this->left_rect_image_pub_->publish(
    this->rectify_image(*image_msg, this->left_map_x_, this->left_map_y_));
}

void StereoCalibrationPublisher::publish_right_camera_info(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg)
{
  this->right_camera_info_.header.stamp = image_msg->header.stamp;
  this->right_rectified_camera_info_.header.stamp = image_msg->header.stamp;
  this->right_camera_info_pub_->publish(this->right_camera_info_);
  this->right_rectified_camera_info_pub_->publish(this->right_rectified_camera_info_);
  this->right_rect_image_pub_->publish(
    this->rectify_image(*image_msg, this->right_map_x_, this->right_map_y_));
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
