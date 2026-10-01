# =====================================================================
 # MIT License
 # 
 # Copyright (c) 2026 Omni Instrument Inc.
 # 
 # Permission is hereby granted, free of charge, to any person obtaining a copy
 # of this software and associated documentation files (the "Software"), to deal
 # in the Software without restriction, including without limitation the rights
 # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 # copies of the Software, and to permit persons to whom the Software is
 # furnished to do so, subject to the following conditions:
 # 
 # The above copyright notice and this permission notice shall be included in
 # all copies or substantial portions of the Software.
 # 
 # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 # THE SOFTWARE.
# ===================================================================== 

cmake_minimum_required(VERSION 3.8)
project(stereo_vio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(stereo_vio_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS calib3d core imgproc video)

add_library(stereo_calibration_publisher_component SHARED
  src/stereo_calibration_publisher.cpp
)

target_include_directories(stereo_calibration_publisher_component PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

target_include_directories(stereo_calibration_publisher_component PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_compile_features(stereo_calibration_publisher_component PUBLIC cxx_std_17)

ament_target_dependencies(stereo_calibration_publisher_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  tf2_ros
)

target_link_libraries(stereo_calibration_publisher_component
  Eigen3::Eigen
  ${OpenCV_LIBS}
)

rclcpp_components_register_nodes(stereo_calibration_publisher_component
  "stereo_vio::StereoCalibrationPublisher"
)

add_library(vio_node_component SHARED
  src/vio_backend.cpp
  src/vio_frontend.cpp
  src/vio_node.cpp
)

target_include_directories(vio_node_component PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

target_include_directories(vio_node_component PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
)

target_compile_features(vio_node_component PUBLIC cxx_std_17)

ament_target_dependencies(vio_node_component
  rclcpp
  rclcpp_components
  sensor_msgs
  nav_msgs
  geometry_msgs
  stereo_vio_msgs
  tf2_ros
  tf2_msgs
  message_filters
)

target_link_libraries(vio_node_component
  ${OpenCV_LIBS}
  gtsam
)

rclcpp_components_register_nodes(vio_node_component
  "stereo_vio::VioNode"
)

add_library(odometry_error_node_component SHARED
  src/odometry_error_node.cpp
)

target_include_directories(odometry_error_node_component PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

target_compile_features(odometry_error_node_component PUBLIC cxx_std_17)

ament_target_dependencies(odometry_error_node_component
  rclcpp
  rclcpp_components
  nav_msgs
  geometry_msgs
  stereo_vio_msgs
  message_filters
)

target_link_libraries(odometry_error_node_component
  Eigen3::Eigen
)

rclcpp_components_register_nodes(odometry_error_node_component
  "stereo_vio::OdometryErrorNode"
)

install(TARGETS
  stereo_calibration_publisher_component
  vio_node_component
  odometry_error_node_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  nav_msgs
  stereo_vio_msgs
  tf2_ros
  tf2_msgs
  message_filters
  Eigen3
  GTSAM
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
