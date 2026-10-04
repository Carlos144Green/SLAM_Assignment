"""
=====================================================================
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
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bag_path = LaunchConfiguration("bag_path")
    calib_file = PathJoinSubstitution(
        [FindPackageShare("stereo_vio"), "config", "calib.yaml"]
    )
    rerun_blueprint_file = PathJoinSubstitution(
        [FindPackageShare("rerun_wrapper"), "template", "rerun_wrapper.rbl"]
    )
    odometry_error_report_file = PathJoinSubstitution(
        [EnvironmentVariable("HOME"), "output", "stereo_vio_odometry_error.txt"]
    )
    stereo_calibration_publisher = ComposableNode(
        package="stereo_vio",
        plugin="stereo_vio::StereoCalibrationPublisher",
        name="stereo_calibration_publisher",
        parameters=[calib_file, {"use_sim_time": True}],
        remappings=[
            ("left/image", "/stereo/left/color/image_raw"),
            ("left/image_rect", "/stereo/left/color/image_rect"),
            ("left/camera_info", "/stereo/left/color/camera_info"),
            ("left/rectified_camera_info", "/stereo/left/color/camera_info_rect"),
            ("right/image", "/stereo/right/color/image_raw"),
            ("right/image_rect", "/stereo/right/color/image_rect"),
            ("right/camera_info", "/stereo/right/color/camera_info"),
            ("right/rectified_camera_info", "/stereo/right/color/camera_info_rect"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    vio_node = ComposableNode(
        package="stereo_vio",
        plugin="stereo_vio::VioNode",
        name="vio_node",
        parameters=[
            {
                "use_sim_time": True,
                "sync_queue_size": 1000,
                "sync_time_seconds": 0.002,
                "pixel_mask_border": 20,
                "base_frame_id": "base_link",
                "imu_frame_id": "oak_imu_link",
                "odom_frame_id": "odom",
            }
        ],
        remappings=[
            ("left/image_rect", "/stereo/left/color/image_rect"),
            ("right/image_rect", "/stereo/right/color/image_rect"),
            ("left/camera_info", "/stereo/left/color/camera_info_rect"),
            ("right/camera_info", "/stereo/right/color/camera_info_rect"),
            ("imu", "/imu/data"),
            ("tf_static", "/tf_static"),
            ("odom", "/stereo_vio/odom"),
            ("observations", "/stereo_vio/observations"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    odometry_error_node = ComposableNode(
        package="stereo_vio",
        plugin="stereo_vio::OdometryErrorNode",
        name="odometry_error_node",
        parameters=[
            {
                "use_sim_time": True,
                "sync_queue_size": 1000,
                "sync_time_seconds": 0.05,
                "error_report_path": odometry_error_report_file,
            }
        ],
        remappings=[
            ("ground_truth/odom", "/ground_truth/odom"),
            ("stereo_vio/odom", "/stereo_vio/odom"),
            ("errors", "/stereo_vio/errors"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    rerun_wrapper_node = ComposableNode(
        package="rerun_wrapper",
        plugin="rerun_wrapper::RerunWrapperNode",
        name="rerun_wrapper_node",
        parameters=[
            {
                "rerun_blueprint_path": rerun_blueprint_file,
            }
        ],
        remappings=[
            ("left/image_rect", "/stereo/left/color/image_rect"),
            ("right/image_rect", "/stereo/right/color/image_rect"),
            ("left/camera_info", "/stereo/left/color/camera_info_rect"),
            ("right/camera_info", "/stereo/right/color/camera_info_rect"),
            ("imu", "/imu/data"),
            ("ground_truth/odom", "/ground_truth/odom"),
            ("stereo_vio/odom", "/stereo_vio/odom"),
            ("stereo_vio/observations", "/stereo_vio/observations"),
            ("stereo_vio/errors", "/stereo_vio/errors"),
            ("tf_static", "/tf_static"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    container = ComposableNodeContainer(
        name="stereo_vio_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            stereo_calibration_publisher,
            vio_node,
            odometry_error_node,
            rerun_wrapper_node,
        ],
    )

    bag_play = ExecuteProcess(
        name="rosbag_play",
        cmd=["ros2", "bag", "play", bag_path, "--clock"],
        output="screen",
    )
    shutdown_after_bag_play = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_play,
            on_exit=[
                TimerAction(
                    period=30.0,
                    actions=[
                        EmitEvent(
                            event=Shutdown(
                                reason="rosbag_play exited; shutting down after 30 seconds"
                            )
                        )
                    ],
                )
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_path",
                default_value=PathJoinSubstitution(
                    [
                        EnvironmentVariable("HOME"),
                        "data",
                        "omni_vio_20260425_220737Z_with_gt",
                    ]
                ),
                description="Path to the ROS 2 bag directory to play.",
            ),
            container,
            bag_play,
            shutdown_after_bag_play,
        ]
    )
