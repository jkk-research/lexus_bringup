# Copyright 2023 Ouster, Inc.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from os.path import join


def generate_launch_description():
    ouster_namespace_arg = DeclareLaunchArgument(
        'ouster_ns',
        default_value='lexus3',
        description='Namespace for the Ouster sensor node')
    ouster_full_namespace_arg = DeclareLaunchArgument(
        'ouster_full_ns',
        default_value=[LaunchConfiguration('ouster_ns'), '/os_left'],
        description='Complete namespace for the ouster driver')

    params_file_arg = DeclareLaunchArgument('params_file',
        default_value=join(
            get_package_share_directory('lexus_bringup'),
            'config',
            'lidar',
            'ouster_config_left.yaml'
        ),
        description='Name or path to the parameter file to use.'
    )

    os_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(
                get_package_share_directory('lexus_bringup'),
                'launch',
                'drivers',
                'os_driver.launch.py')
        )
    )

    return LaunchDescription([
        ouster_namespace_arg,
        ouster_full_namespace_arg,
        params_file_arg,

        os_launcher
    ])
 