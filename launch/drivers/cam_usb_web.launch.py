from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from os.path import join


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='lexus3/camera/usb_web',
        description='Namespace argument for usb web camera node'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=join(
            get_package_share_directory('lexus_bringup'),
            'config',
            'camera',
            'usb_web_params.yaml'
        ),
        description='Name or path to the parameter file to use.'
    )

    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            LaunchConfiguration('params_file')
        ]
    )

    return LaunchDescription([
        namespace_arg,
        params_file_arg,

        camera_node
    ])