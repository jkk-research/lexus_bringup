# Copyright 2023 Ouster, Inc.
#

"""Launch a sensor node along with..."""

from pathlib import Path
import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, GroupAction, TimerAction, ExecuteProcess)
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import PushRosNamespace, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """
    Generate launch description for running ouster_ros components separately each
    component will run in a separate process).
    """
    aw_pkg_dir = get_package_share_directory('autoware_launch')
    default_params_file = \
        Path(aw_pkg_dir) / 'launch' / 'lidar_config_comp.yaml'
    params_file = LaunchConfiguration('params_file')
    params_file_arg = DeclareLaunchArgument('params_file',
                                            default_value=str(
                                                default_params_file),
                                            description='name or path to the parameters file to use.')
    
    container_name_arg = DeclareLaunchArgument('pointcloud_container_name', default_value='pointcloud_container')

    container = Node(
        name=LaunchConfiguration('pointcloud_container_name'),
        package='rclcpp_components',
        executable='component_container',
        output='both'
    )

    load_composable_nodes = LoadComposableNodes(
        target_container=LaunchConfiguration('pointcloud_container_name'),
        composable_node_descriptions=[
            ComposableNode(
                package='ouster_ros',
                plugin='ouster_ros::OusterSensor',
                name="os_sensor",
                namespace="",
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='ouster_ros',
                plugin='ouster_ros::OusterCloud',
                name="os_cloud",
                namespace="",
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        ],
    )
    
    def invoke_lifecycle_cmd(node_name, verb):
        ros2_exec = FindExecutable(name='ros2')
        return ExecuteProcess(
            cmd=[[ros2_exec, ' lifecycle set /lexus3/os_left/os_sensor ', verb]],
            shell=True)

    sensor_configure_cmd = invoke_lifecycle_cmd('os_sensor', 'configure')
    sensor_activate_cmd = invoke_lifecycle_cmd('os_sensor', 'activate')

    return launch.LaunchDescription([
        params_file_arg,
        container_name_arg,
        container,
        GroupAction(
            actions=[
                PushRosNamespace('lexus3/os_left'),
                load_composable_nodes
            ]
        ),
        sensor_configure_cmd,
        TimerAction(period=1.0, actions=[sensor_activate_cmd])
    ])
