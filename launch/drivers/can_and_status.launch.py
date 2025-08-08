from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    pacmod_namespace_arg = DeclareLaunchArgument(
        'pacmod_namespace',
        default_value='lexus3',
        description='Namespace for pacmod3 and vehicle status')

    pacmod_launcher = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([
            FindPackageShare("lexus_bringup"),
            '/launch/drivers',
            '/can_pacmod3.launch.xml'
        ]),
        launch_arguments={
            'namespace': [LaunchConfiguration('pacmod_namespace'), '/pacmod']
        }.items()
    )

    vehicle_status_node = Node(
        package='lexus_bringup',
        executable='vehicle_status_from_pacmod',
        output='screen',
        namespace=LaunchConfiguration('pacmod_namespace'),
        parameters=[{
            "steering_topic": "pacmod/steering_rpt",
            "speed_topic": "pacmod/vehicle_speed_rpt",
            "status_topic": "vehicle_status"
        }],
    )

    return LaunchDescription([
        pacmod_namespace_arg,
        pacmod_launcher,
        vehicle_status_node
    ])
 