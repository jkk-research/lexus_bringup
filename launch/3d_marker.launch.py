from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    #pkg_name = 'lexus_base'
    #pkg_dir = os.popen('/bin/bash -c "source /usr/share/colcon_cd/function/colcon_cd.sh && colcon_cd %s && pwd"' % pkg_name).read().strip()
    #print(pkg_dir)

    return LaunchDescription([
        Node(
            package='rviz_markers',
            #namespace='lexus3',
            executable='lexus',
            name='lexus_marker_3d',
            output='screen',
            parameters=[{"lexus_frame_id":'lexus3/base_link'}],
        )
    ])