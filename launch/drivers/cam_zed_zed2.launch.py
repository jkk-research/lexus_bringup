import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    zed_ns_arg = DeclareLaunchArgument(
        'zed_ns',
        default_value='lexus3/camera/zed',
        description='Full ROS namespace for the ZED camera'
    )

    zed_camera_name_arg = DeclareLaunchArgument(
        'zed_camera_name',
        default_value='lexus3_zed',
        description='Logical camera name for ZED parameters'
    )

    camera_model = 'zed2i'

    zed_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lexus_bringup'),
                'launch',
                'drivers',
                'cam_zed_driver.launch.py'
            )
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('zed_ns'),
            'camera_name': LaunchConfiguration('zed_camera_name'),
            'camera_model': camera_model,
        }.items()
    )

    return LaunchDescription([
        zed_ns_arg,
        zed_camera_name_arg,
        zed_driver_launch
    ])