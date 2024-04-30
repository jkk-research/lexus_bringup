# Copyright 2023 Ouster, Inc.
#

"""Launch a sensor node along with..."""

from pathlib import Path
import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, GroupAction, TimerAction, ExecuteProcess)
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import PushRosNamespace, LoadComposableNodes, Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """
    Generate launch description for running ouster_ros components separately each
    component will run in a separate process).
    """
    node_name = "os_all"
    namespace = "/lexus3"
    aw_pkg_dir = get_package_share_directory('lexus_bringup')
    default_params_file = \
        Path(aw_pkg_dir) / 'launch' / 'drivers' / 'ouster_config_a.yaml'
    params_file = LaunchConfiguration('params_file')
    params_file_arg = DeclareLaunchArgument('params_file',
                                            default_value=str(
                                                default_params_file),
                                            description='name or path to the parameters file to use.')

    container_name_arg = DeclareLaunchArgument('pointcloud_container_name', default_value='pointcloud_container')

    container = ComposableNodeContainer(
        name=LaunchConfiguration('pointcloud_container_name'),
        package='rclcpp_components',
        executable='component_container',
        namespace="",
        output='both',
        composable_node_descriptions=[
            ComposableNode(
                package='ros2_ouster',
                plugin='ros2_ouster::Driver',
                name="os_right",
                parameters=[params_file],
                remappings=[('/points', namespace + "/os_right" + '/points')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='ros2_ouster',
                plugin='ros2_ouster::Driver',
                name="os_left",
                parameters=[params_file],
                remappings=[('/points', namespace + "/os_left" + '/points')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='ros2_ouster',
                plugin='ros2_ouster::Driver',
                name="os_center",
                parameters=[params_file],
                remappings=[('/points', namespace + "/os_center" + '/points')],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        ]
    )

    static_tf = Node(
        package='tf2_ros',
        #namespace='lexus3',
        executable='static_transform_publisher',
        name=node_name + '_180_tf_publisher',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '3.14159265359', '0.0', '0.0', namespace + '/' + node_name +'_a', namespace + '/' + node_name + '_180'],
    )

    def invoke_lifecycle_cmd(node_name, verb):
        ros2_exec = FindExecutable(name='ros2')
        return ExecuteProcess(
            cmd=[[ros2_exec, ' lifecycle set /', node_name, ' ', verb]],
            shell=True)

    sensor_right_configure_cmd = invoke_lifecycle_cmd('os_right', 'configure')
    sensor_right_activate_cmd = invoke_lifecycle_cmd('os_right', 'activate')
    sensor_left_configure_cmd = invoke_lifecycle_cmd('os_left', 'configure')
    sensor_left_activate_cmd = invoke_lifecycle_cmd('os_left', 'activate')
    sensor_center_configure_cmd = invoke_lifecycle_cmd('os_center', 'configure')
    sensor_center_activate_cmd = invoke_lifecycle_cmd('os_center', 'activate')

    return launch.LaunchDescription([
        params_file_arg,
        container_name_arg,
        container,
        sensor_right_configure_cmd,
        TimerAction(period=1.0, actions=[sensor_right_activate_cmd]),
        sensor_left_configure_cmd,
        TimerAction(period=60.0, actions=[sensor_left_activate_cmd]),
        sensor_center_configure_cmd,
        TimerAction(period=120.0, actions=[sensor_center_activate_cmd])
        # static_tf
    ])
