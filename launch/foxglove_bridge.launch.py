from launch import LaunchDescription
from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# from math import pi, degrees, radians


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            parameters=[
                {'port': 8765},
                {'address': '0.0.0.0'},
                {'tls': False},
                {'certfile': ''},
                {'keyfile': ''},
                #{'topic_whitelist': "'.*'"},
                {'max_qos_depth': 10},
                {'num_threads': 0},
                {'use_sim_time': False},
                {'send_buffer_limit': 100000000 }, # 100MB because of the large point cloud
            ]
        ),
    ])