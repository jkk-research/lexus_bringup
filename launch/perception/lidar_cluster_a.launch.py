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
            executable='euclidean_grid',
            output='screen',
            parameters=[
                {'points_in_topic': LaunchConfiguration("topic")},
                {'points_out_topic': 'clustered_points'},
                {'marker_out_topic': 'clustered_marker'},
                {'tolerance': 2.0},
                {'max_cluster_size': 4000},
                {'voxel_leaf_size': 4.0},
                {'min_points_number_per_voxel': 5},
                {'verbose1': True},
                {'verbose2': True},
            ]
        )
    ])