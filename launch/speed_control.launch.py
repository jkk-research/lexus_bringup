from launch import LaunchDescription
from launch_ros.actions import Node

ns_vehicle = 'lexus3'

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lexus_bringup',
            namespace= ns_vehicle,
            executable='speed_control',
            name='speed_control_pid_longitudinal',
            parameters=[
                {"p_gain_accel": 0.29},
                {"i_gain_accel": 0.035},
                {"d_gain_accel": 0.0},
                {"p_gain_brake": 0.15},
                {"i_gain_brake": 0.038},
                {"d_gain_brake": 0.0},
        ]
        )
    ])