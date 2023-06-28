# Copyright 2022 Stereolabs
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

from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, GroupAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    # Camera model (force value)
    camera_model = 'zed2i'
    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(
                namespace="lexus3",
            ),

            # ZED Wrapper node
            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource([
                    get_package_share_directory('lexus_bringup'),
                    '/launch/drivers/zed_default_a_common.launch.py'
                ]),
                launch_arguments={
                    'camera_model': camera_model,
                }.items()
            ),
        ]
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    # Add nodes to LaunchDescription
    # ld.add_action(zed_wrapper_launch)

    return ld
