# Copyright 2023 Ouster, Inc.

import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from os.path import join


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=join(
            get_package_share_directory('lexus_bringup'),
            'launch',
            'drivers',
            'ouster_config_c.yaml'
        ),
        description='Name or path to the parameter file to use.'
    )
    ouster_container_namespace_arg = DeclareLaunchArgument(
        'ouster_container_ns',
        default_value='lexus3',
        description='Namespace for the Ouster sensor container'
    )
    ouster_ns_right_arg = DeclareLaunchArgument(
        'ouster_right_ns',
        default_value='lexus3/os_right',
        description='Namespace for the Ouster right sensor node'
    )
    ouster_ns_center_arg = DeclareLaunchArgument(
        'ouster_center_ns',
        default_value='lexus3/os_center',
        description='Namespace for the Ouster center sensor node'
    )
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='True',
        description='Whether to automatically start the Ouster sensor'
    )


    os_center_sensor = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterSensor',
        name='os_sensor',
        namespace=LaunchConfiguration('ouster_center_ns'),
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'auto_start': LaunchConfiguration('auto_start')
            }
        ]
    )

    os_center_cloud = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterCloud',
        name='os_cloud',
        namespace=LaunchConfiguration('ouster_center_ns'),
        parameters=[LaunchConfiguration('params_file')]
    )


    os_right_sensor = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterSensor',
        name='os_sensor',
        namespace=LaunchConfiguration('ouster_right_ns'),
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'auto_start': LaunchConfiguration('auto_start')
            }
        ]
    )

    os_right_cloud = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterCloud',
        name='os_cloud',
        namespace=LaunchConfiguration('ouster_right_ns'),
        parameters=[LaunchConfiguration('params_file')]
    )

    os_container = ComposableNodeContainer(
        name='os_container',
        namespace=LaunchConfiguration('ouster_container_ns'),
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
    )

    load_composable_nodes = LoadComposableNodes(
        target_container=[LaunchConfiguration('ouster_container_ns'), '/os_container'],
        composable_node_descriptions=[
            os_center_sensor,
            os_center_cloud,
            os_right_sensor,
            os_right_cloud
    
        ])

    # def invoke_lifecycle_cmd(node_name, verb):
    #     ros2_exec = FindExecutable(name='ros2')
    #     return ExecuteProcess(
    #         cmd=[[ros2_exec, ' lifecycle set /', node_name, ' ', verb]],
    #         shell=True
    #     )

    # sensor_right_configure_cmd = invoke_lifecycle_cmd('lexus3/os_right/os_driver', 'configure')
    # sensor_right_activate_cmd = invoke_lifecycle_cmd('lexus3/os_right/os_driver', 'activate')
    # sensor_center_configure_cmd = invoke_lifecycle_cmd('lexus3/os_center/os_driver', 'configure')
    # sensor_center_activate_cmd = invoke_lifecycle_cmd('lexus3/os_center/os_driver', 'activate')
    


    return launch.LaunchDescription([
        params_file_arg,
        ouster_container_namespace_arg,
        ouster_ns_right_arg,
        ouster_ns_center_arg,
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
