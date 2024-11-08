# Copyright 2023 Ouster, Inc.
#

"""Launch ouster nodes using a composite container"""

from pathlib import Path
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            ExecuteProcess, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import LoadComposableNodes


def generate_launch_description():
    """
    Generate launch description for running ouster_ros components in a single
    process/container.
    """
    ouster_ros_pkg_dir = get_package_share_directory('ouster_ros')
    lex_pkg_dir = get_package_share_directory('lexus_bringup')
    default_params_file = Path(lex_pkg_dir) / 'launch' / 'drivers' / 'ouster_config_c.yaml'
    params_file = LaunchConfiguration('params_file')
    params_file_arg = DeclareLaunchArgument('params_file',
                                            default_value=str(
                                                default_params_file),
                                            description='name or path to the parameters file to use.')

    ouster_right_ns = LaunchConfiguration('ouster_right_ns')
    ouster_center_ns = LaunchConfiguration('ouster_center_ns')
    ouster_ns_right_arg = DeclareLaunchArgument('ouster_right_ns', default_value='lexus3/os_right')
    ouster_ns_center_arg = DeclareLaunchArgument('ouster_center_ns', default_value='lexus3/os_center')

    rviz_enable = LaunchConfiguration('viz')
    rviz_enable_arg = DeclareLaunchArgument('viz', default_value='False')

    auto_start = LaunchConfiguration('auto_start')
    auto_start_arg = DeclareLaunchArgument('auto_start', default_value='True')

    os_center_sensor = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterSensor',
        name='os_sensor',
        namespace=ouster_center_ns,
        parameters=[params_file,
        {'auto_start': auto_start}]
    )

    os_center_cloud = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterCloud',
        name='os_cloud',
        namespace=ouster_center_ns,
        parameters=[params_file]
    )


    os_right_sensor = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterSensor',
        name='os_sensor',
        namespace=ouster_right_ns,
        parameters=[params_file,
        {'auto_start': auto_start}]
    )

    os_right_cloud = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterCloud',
        name='os_cloud',
        namespace=ouster_right_ns,
        parameters=[params_file]
    )


    # os_container_right = ComposableNodeContainer(
    #     name='os_container_right',
    #     namespace='a',
    #     package='rclcpp_components',
    #     executable='component_container_mt',
    #     composable_node_descriptions=[
    #         os_right_sensor,
    #         os_right_cloud,
    #     ],
    #     output='screen',
    # )

    os_container = ComposableNodeContainer(
        name='os_container_center',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        # composable_node_descriptions=[
        #     os_center_sensor,
        #     os_center_cloud,
        #     os_right_sensor,
        #     os_right_cloud
        # ],
        output='screen',
    )

    load_composable_nodes = LoadComposableNodes(
        target_container='os_container_center',
        composable_node_descriptions=[
            os_center_sensor,
            os_center_cloud,
            os_right_sensor,
            os_right_cloud
    
        ])


    def invoke_lifecycle_cmd(node_name, verb):
        ros2_exec = FindExecutable(name='ros2')
        return ExecuteProcess(
            cmd=[[ros2_exec, ' lifecycle set /', node_name, ' ', verb]],
            shell=True
        )

    # sensor_right_configure_cmd = invoke_lifecycle_cmd('lexus3/os_right/os_driver', 'configure')
    # sensor_right_activate_cmd = invoke_lifecycle_cmd('lexus3/os_right/os_driver', 'activate')
    # sensor_center_configure_cmd = invoke_lifecycle_cmd('lexus3/os_center/os_driver', 'configure')
    # sensor_center_activate_cmd = invoke_lifecycle_cmd('lexus3/os_center/os_driver', 'activate')
    


    return launch.LaunchDescription([
        params_file_arg,
        ouster_ns_right_arg,
        ouster_ns_center_arg,
        rviz_enable_arg,
        auto_start_arg,
        os_container,
        load_composable_nodes,
        #os_container_right,
        #os_container_center,
        # TimerAction(period=4.0, actions=[sensor_left_configure_cmd]),
        # TimerAction(period=8.0, actions=[sensor_left_activate_cmd]),
        # TimerAction(period=12.0, actions=[sensor_right_configure_cmd]),
        # TimerAction(period=16.0, actions=[sensor_right_activate_cmd]),
        # TimerAction(period=20.0, actions=[sensor_center_configure_cmd]),
        # TimerAction(period=24.0, actions=[sensor_center_activate_cmd]),
    ])
