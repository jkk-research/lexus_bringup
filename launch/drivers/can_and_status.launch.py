from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


ns_vehicle = "lexus3"

def generate_launch_description():
    return LaunchDescription([

        IncludeLaunchDescription(
            XMLLaunchDescriptionSource([
                FindPackageShare("lexus_bringup"), '/launch/drivers', '/can_pacmod3.launch.xml'])
        ),

        Node(
            package='lexus_bringup',
            executable='vehicle_status_from_pacmod',
            output='screen',
            namespace = ns_vehicle,
            parameters=[
                {"steering_topic": "pacmod/steering_rpt"},
                {"speed_topic": "pacmod/vehicle_speed_rpt"},
                {"status_topic": "vehicle_status"},
            ],
        ),
    ])
