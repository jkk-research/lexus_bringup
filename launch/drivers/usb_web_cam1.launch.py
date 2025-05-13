from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os

# lexus_bringup_pkg_dir = get_package_share_directory('lexus_bringup')
lexus_bringup_pkg_dir = os.path.join(
    '/home/dev/ros2_ws/src/lexus_bringup', # TODO: change this to not hardcoded path
)
print(lexus_bringup_pkg_dir)

def generate_launch_description():
    params_file = os.path.join(
        lexus_bringup_pkg_dir, 'launch', 'drivers', 'usb_cam_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[params_file]
        )
    ])
