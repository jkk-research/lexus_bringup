from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():
    ros_sample_dir = get_package_share_directory('luminar_driver')

    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_id', default_value='luminar_lidar_151',
        ),
        DeclareLaunchArgument(
            'lidar_data_port', default_value='4370',
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
        ),
        DeclareLaunchArgument(
            'combined_config', default_value=f'{ros_sample_dir}/rviz/combined_mode.rviz',
        ),
        Node(
            package='luminar_driver',
            executable='luminar_driver',
            name='luminar_driver',
            output='screen',
            parameters=[
                {'lidar_data_port': LaunchConfiguration('lidar_data_port')},
            ],
        ),
        Node(
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_rviz'), "'=='true'"])),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('combined_config')],
        ),
    ])
