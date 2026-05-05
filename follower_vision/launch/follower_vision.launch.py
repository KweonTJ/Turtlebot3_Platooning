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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("follower_vision"),
        "config",
        "vision_params.yaml",
    )

    use_debug_image = LaunchConfiguration("use_debug_image")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_debug_image", default_value="true"),
            Node(
                package="follower_vision",
                executable="follower_vision_node",
                name="follower_vision",
                output="screen",
                parameters=[
                    params_file,
                    {"publish_debug_image": use_debug_image},
                ],
            ),
        ]
    )
