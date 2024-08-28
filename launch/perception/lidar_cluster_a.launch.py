from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():



    return LaunchDescription([
        DeclareLaunchArgument("topic", description="a pointcloud topic to process", default_value="nonground"),
        Node(
            package='lidar_cluster',
            executable='dbscan_spatial',
            output='screen',
            parameters=[
                {'points_in_topic': LaunchConfiguration("topic")},
                {'points_out_topic': 'clustered_points'},
                {'marker_out_topic': 'clustered_marker'},
                {'verbose1': True},
                {'verbose2': True},
            ]
        )
    ])