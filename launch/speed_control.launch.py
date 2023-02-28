from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    pkg_name = 'lexus_bringup'
    pkg_dir = os.popen('/bin/bash -c "source /usr/share/colcon_cd/function/colcon_cd.sh && \
        colcon_cd %s && pwd"' % pkg_name).read().strip()


    return LaunchDescription([
        Node(
            package='lexus_bringup',
            namespace='',
            executable='speed_control',
            name='speed_control_pid_longitudinal',
            parameters=[
                {"p_gain_accel": 15.0},
                {"i_gain_accel": 0.0},
                {"d_gain_accel": 0.0},
                {"p_gain_brake": 9.0},
                {"i_gain_brake": 0.0},
                {"d_gain_brake": 0.0}
        ]
        )
    ])