from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='rviz_markers',
            executable='lexus',
            name='lexus_marker_3d',
            output='screen',
            parameters=[{"lexus_frame_id":'lexus3/base_link'}],
        )
    ])