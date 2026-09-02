from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():


    # radar
    radar_publish_debug_arg = DeclareLaunchArgument(
        'radar/settings/publish_debug',
        default_value='true',
        description='Enable or disable debug publishing'
    )


    radar_driver_fc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(
                get_package_share_directory('rd6_driver'),
                'launch',
                'rd6.launch.py')
        ),
        launch_arguments={
            'radar_config_file' : join(get_package_share_directory('lexus_bringup'),'config','radar','fcRadarParams.yaml'),
            'radar_interface'   : 'can4',
            'radar_namespace'   : '/sensing/radar/fc',
            'radar_locations_frame_id' : 'radar_front_center',
            'publish_debug'   : LaunchConfiguration('radar/settings/publish_debug')
        }.items()
    )


    radar_driver_fl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(
                get_package_share_directory('rd6_driver'),
                'launch',
                'rd6.launch.py')
        ),
        launch_arguments={
            'radar_config_file' : join(get_package_share_directory('lexus_bringup'),'config','radar','flRadarParams.yaml'),
            'radar_interface'   : 'can2',
            'radar_namespace'   : '/sensing/radar/fl',
            'radar_locations_frame_id' : 'radar_front_left',
            'publish_debug'   : LaunchConfiguration('radar/settings/publish_debug')
        }.items()
    )

    radar_driver_fr = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(
                get_package_share_directory('rd6_driver'),
                'launch',
                'rd6.launch.py')
        ),
        launch_arguments={
            'radar_config_file' : join(get_package_share_directory('lexus_bringup'),'config','radar','frRadarParams.yaml'),
            'radar_interface'   : 'can3',
            'radar_namespace'   : '/sensing/radar/fr',
            'radar_locations_frame_id' : 'radar_front_right',
            'publish_debug'   : LaunchConfiguration('radar/settings/publish_debug')
        }.items()
    )

    radar_driver_rl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(
                get_package_share_directory('rd6_driver'),
                'launch',
                'rd6.launch.py')
        ),
        launch_arguments={
            'radar_config_file' : join(get_package_share_directory('lexus_bringup'),'config','radar','rlRadarParams.yaml'),
            'radar_interface'   : 'can0',
            'radar_namespace'   : '/sensing/radar/rl',
            'radar_locations_frame_id' : 'radar_rear_left',
            'publish_debug'   : LaunchConfiguration('radar/settings/publish_debug')
        }.items()
    )

    radar_driver_rr = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(
                get_package_share_directory('rd6_driver'),
                'launch',
                'rd6.launch.py')
        ),
        launch_arguments={
            'radar_config_file' : join(get_package_share_directory('lexus_bringup'),'config','radar','rrRadarParams.yaml'),
            'radar_interface'   : 'can1',
            'radar_namespace'   : '/sensing/radar/rr',
            'radar_locations_frame_id' : 'radar_rear_right',
            'publish_debug'   : LaunchConfiguration('radar/settings/publish_debug')
        }.items()

    )


    return LaunchDescription([

        radar_publish_debug_arg,
        radar_driver_fc,
        radar_driver_fl,
        radar_driver_fr,
        radar_driver_rl,
        radar_driver_rr,

    ])


