# Copyright 2023 Ouster, Inc.
#

"""Launch a sensor node along with..."""

from pathlib import Path
import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, GroupAction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """
    Generate launch description for running ouster_ros components separately each
    component will run in a separate process).
    """
    aw_pkg_dir = get_package_share_directory('autoware_launch')
    default_params_file = \
        Path(aw_pkg_dir) / 'launch' / 'lidar_config.yaml'
    params_file = LaunchConfiguration('params_file')
    params_file_arg = DeclareLaunchArgument('params_file',
                                            default_value=str(
                                                default_params_file),
                                            description='name or path to the parameters file to use.')

    container_name_arg = DeclareLaunchArgument('pointcloud_container_name', default_value='pointcloud_container')

    load_composable_nodes = LoadComposableNodes(
        target_container=LaunchConfiguration('pointcloud_container_name'),
        composable_node_descriptions=[
            ComposableNode(
                package='ouster_ros',
                plugin='ouster_ros::OusterDriver',
                name="os_driver_right",
                namespace="",
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
    )

    return launch.LaunchDescription([
        GroupAction(
            actions=[
                PushRosNamespace('lexus3/os_right'),
                params_file_arg,
                container_name_arg,
                load_composable_nodes
            ]
        )
    ])
