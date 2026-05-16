# Copyright 2026 ktj
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch.substitutions import EnvironmentVariable
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_camera = LaunchConfiguration("use_camera")
    start_vision = LaunchConfiguration("start_vision")
    video_device = LaunchConfiguration("video_device")
    use_debug_image = LaunchConfiguration("use_debug_image")
    publish_robot_description = LaunchConfiguration("publish_robot_description")
    use_static_urdf = LaunchConfiguration("use_static_urdf")
    robot_prefix = LaunchConfiguration("robot_prefix")
    camera_frame_id = LaunchConfiguration("camera_frame_id")
    start_rviz = LaunchConfiguration("start_rviz")
    start_joint_state_publisher = LaunchConfiguration("start_joint_state_publisher")
    start_base_driver = LaunchConfiguration("start_base_driver")
    start_platooning = LaunchConfiguration("start_platooning")
    start_safety = LaunchConfiguration("start_safety")
    usb_port = LaunchConfiguration("usb_port")
    tb3_param_dir = LaunchConfiguration("tb3_param_dir")
    start_monitor_uploader = LaunchConfiguration("start_monitor_uploader")
    monitor_server = LaunchConfiguration("monitor_server")
    monitor_token = LaunchConfiguration("monitor_token")
    monitor_video_enabled = LaunchConfiguration("monitor_video_enabled")
    monitor_status_period = LaunchConfiguration("monitor_status_period")
    monitor_video_period = LaunchConfiguration("monitor_video_period")
    monitor_jpeg_quality = LaunchConfiguration("monitor_jpeg_quality")
    monitor_image_width = LaunchConfiguration("monitor_image_width")
    monitor_image_height = LaunchConfiguration("monitor_image_height")
    monitor_http_timeout = LaunchConfiguration("monitor_http_timeout")

    vision_params = PathJoinSubstitution(
        [
            FindPackageShare("follower_vision"),
            "config",
            "vision_params.yaml",
        ]
    )
    platooning_params = os.path.join(
        get_package_share_directory("follower_platooning"),
        "config",
        "platooning_params.yaml",
    )
    safety_params = os.path.join(
        get_package_share_directory("follower_safety"),
        "config",
        "safety_params.yaml",
    )
    rviz_config = os.path.join(
        get_package_share_directory("follower_bringup"),
        "rviz",
        "platooning_model.rviz",
    )
    default_tb3_param = os.path.join(
        get_package_share_directory("turtlebot3_bringup"),
        "param",
        "humble",
        "waffle_pi.yaml",
    )

    platooning_xacro_file = PathJoinSubstitution(
        [
            FindPackageShare("follower_bringup"),
            "urdf",
            "turtlebot3_platooning.urdf.xacro",
        ]
    )
    platooning_urdf_file = PathJoinSubstitution(
        [
            FindPackageShare("turtlebot3_manipulation_description"),
            "urdf",
            "turtlebot3_platooning.urdf",
        ]
    )

    xacro_robot_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                platooning_xacro_file,
                " ",
                "prefix:=",
                robot_prefix,
            ]
        ),
        value_type=str,
    )
    urdf_robot_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="cat")]),
                " ",
                platooning_urdf_file,
            ]
        ),
        value_type=str,
    )

    use_xacro_description = IfCondition(
        PythonExpression(
            [
                "'",
                publish_robot_description,
                "'.lower() == 'true' and '",
                use_static_urdf,
                "'.lower() == 'false'",
            ]
        )
    )
    use_urdf_description = IfCondition(
        PythonExpression(
            [
                "'",
                publish_robot_description,
                "'.lower() == 'true' and '",
                use_static_urdf,
                "'.lower() == 'true'",
            ]
        )
    )
    use_xacro_joint_state_publisher = IfCondition(
        PythonExpression(
            [
                "'",
                publish_robot_description,
                "'.lower() == 'true' and '",
                use_static_urdf,
                "'.lower() == 'false' and '",
                start_joint_state_publisher,
                "'.lower() == 'true'",
            ]
        )
    )
    use_urdf_joint_state_publisher = IfCondition(
        PythonExpression(
            [
                "'",
                publish_robot_description,
                "'.lower() == 'true' and '",
                use_static_urdf,
                "'.lower() == 'true' and '",
                start_joint_state_publisher,
                "'.lower() == 'true'",
            ]
        )
    )
    use_platooning_with_safety = IfCondition(
        PythonExpression(
            [
                "'",
                start_platooning,
                "'.lower() == 'true' and '",
                start_safety,
                "'.lower() == 'true'",
            ]
        )
    )
    use_platooning_direct = IfCondition(
        PythonExpression(
            [
                "'",
                start_platooning,
                "'.lower() == 'true' and '",
                start_safety,
                "'.lower() == 'false'",
            ]
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_camera", default_value="true"),
            DeclareLaunchArgument("start_vision", default_value="false"),
            DeclareLaunchArgument("video_device", default_value="/dev/video0"),
            DeclareLaunchArgument("use_debug_image", default_value="true"),
            DeclareLaunchArgument("publish_robot_description", default_value="true"),
            DeclareLaunchArgument("use_static_urdf", default_value="false"),
            DeclareLaunchArgument("robot_prefix", default_value=""),
            DeclareLaunchArgument("camera_frame_id", default_value="camera_rgb_optical_frame"),
            DeclareLaunchArgument("start_rviz", default_value="false"),
            DeclareLaunchArgument("start_joint_state_publisher", default_value="false"),
            DeclareLaunchArgument("start_base_driver", default_value="true"),
            DeclareLaunchArgument("start_platooning", default_value="true"),
            DeclareLaunchArgument("start_safety", default_value="false"),
            DeclareLaunchArgument("usb_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("tb3_param_dir", default_value=default_tb3_param),
            DeclareLaunchArgument("start_monitor_uploader", default_value="true"),
            DeclareLaunchArgument(
                "monitor_server",
                default_value=EnvironmentVariable(
                    "MONITOR_SERVER_URL",
                    default_value="http://192.168.0.13:8000",
                ),
            ),
            DeclareLaunchArgument(
                "monitor_token",
                default_value=EnvironmentVariable("MONITOR_TOKEN", default_value=""),
            ),
            DeclareLaunchArgument("monitor_video_enabled", default_value="true"),
            DeclareLaunchArgument("monitor_status_period", default_value="0.2"),
            DeclareLaunchArgument("monitor_video_period", default_value="0.25"),
            DeclareLaunchArgument("monitor_jpeg_quality", default_value="65"),
            DeclareLaunchArgument("monitor_image_width", default_value="640"),
            DeclareLaunchArgument("monitor_image_height", default_value="480"),
            DeclareLaunchArgument("monitor_http_timeout", default_value="1.0"),
            Node(
                package="turtlebot3_node",
                executable="turtlebot3_ros",
                output="screen",
                condition=IfCondition(start_base_driver),
                parameters=[tb3_param_dir, {"namespace": ""}],
                arguments=["-i", usb_port],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                condition=use_xacro_description,
                parameters=[{"robot_description": xacro_robot_description}],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                condition=use_urdf_description,
                parameters=[{"robot_description": urdf_robot_description}],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                condition=use_xacro_joint_state_publisher,
                parameters=[{"robot_description": xacro_robot_description}],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                condition=use_urdf_joint_state_publisher,
                parameters=[{"robot_description": urdf_robot_description}],
            ),
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                name="follower_camera",
                output="screen",
                condition=IfCondition(use_camera),
                parameters=[
                    {
                        "video_device": video_device,
                        "camera_frame_id": camera_frame_id,
                    },
                ],
                remappings=[
                    ("image_raw", "/follower/camera/image_raw"),
                    ("camera_info", "/follower/camera/camera_info"),
                ],
            ),
            Node(
                package="follower_vision",
                executable="follower_vision_node",
                name="follower_vision",
                output="screen",
                condition=IfCondition(start_vision),
                parameters=[
                    vision_params,
                    {"publish_debug_image": use_debug_image},
                ],
            ),
            Node(
                package="follower_platooning",
                executable="leader_odom_aligner_node",
                name="leader_odom_aligner",
                output="screen",
                condition=IfCondition(start_platooning),
                parameters=[platooning_params],
            ),
            Node(
                package="follower_platooning",
                executable="follower_platooning_node",
                name="follower_platooning",
                output="screen",
                condition=use_platooning_with_safety,
                parameters=[platooning_params],
            ),
            Node(
                package="follower_platooning",
                executable="follower_platooning_node",
                name="follower_platooning",
                output="screen",
                condition=use_platooning_direct,
                parameters=[platooning_params, {"cmd_vel_raw_topic": "/cmd_vel"}],
            ),
            Node(
                package="follower_safety",
                executable="follower_safety_node",
                name="follower_safety",
                output="screen",
                condition=IfCondition(start_safety),
                parameters=[safety_params],
            ),
            Node(
                package="follower_platooning",
                executable="robot_status_uploader.py",
                name="follower_status_uploader",
                output="screen",
                arguments=[
                    "--robot", "follower",
                    "--server", monitor_server,
                    "--token", monitor_token,
                    "--status-period", monitor_status_period,
                    "--video-period", monitor_video_period,
                    "--jpeg-quality", monitor_jpeg_quality,
                    "--image-width", monitor_image_width,
                    "--image-height", monitor_image_height,
                    "--http-timeout", monitor_http_timeout,
                    "--video-enabled", monitor_video_enabled,
                ],
                condition=IfCondition(start_monitor_uploader),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(start_rviz),
            ),
        ]
    )
