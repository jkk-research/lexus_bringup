from launch import LaunchDescription
from launch_ros.actions import Node
import math
import os

def generate_launch_description():
    ns_vehicle = "lexus3"

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gyor0_tf_publisher',
            output='screen',
            arguments=[
                '--x',  '697237.0',
                '--y',  '5285644.0',
                '--z',  '0.0',
                '--qx', '0.0',
                '--qy', '0.0',
                '--qz', '0.0',
                '--qw', '1.0',

                '--frame-id',      'map',
                '--child-frame-id','map_gyor_0'
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='zala0_tf_publisher',
            output='screen',
            arguments=[
                '--x',  '639770.0',
                '--y',  '5195040.0',
                '--z',  '0.0',
                '--qx', '0.0',
                '--qy', '0.0',
                '--qz', '0.0',
                '--qw', '1.0',

                '--frame-id',       'map',
                '--child-frame-id', 'map_zala_0'
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='duro_gps_tf_publisher',
            output='screen',
            arguments=[
                '--x',  '1.6',
                '--y',  '0.0',
                '--z',  '0.2',
                '--qx', '0.0',
                '--qy', '0.0',
                '--qz', '0.0',
                '--qw', '1.0',

                '--frame-id',       ns_vehicle + '/' + 'base_link',
                '--child-frame-id', ns_vehicle + '/' + 'duro_gps'
            ],
        ),
        ## moved to separate gps launch files TODO: delete
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_gps_tf_publisher',
        #     output='screen',
        #     # https://raw.githubusercontent.com/wiki/szenergy/szenergy-public-resources/img/2022.L.01.svg

        #     arguments=[
        #         '--x',     '-1.542',
        #         '--y',     '-0.49',
        #         '--z',     '-1.479',
        #         '--yaw',   '0.0',
        #         '--pitch', '0.0',
        #         '--roll',  '0.0',

        #         '--frame-id',       ns_vehicle + '/' + 'gps',
        #         '--child-frame-id', ns_vehicle + '/' + 'base_link'
        #     ],
        # ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='zed_camera_front_tf_publisher',
            output='screen',
            arguments=[
                '--x',  '1.6',
                '--y',  '0.0',
                '--z',  '1.286',
                '--qx', '0.0',
                '--qy', '0.0',
                '--qz', '0.0',
                '--qw', '1.0',

                '--frame-id',       ns_vehicle + '/' + 'base_link',
                '--child-frame-id', ns_vehicle + '/' +'_left_camera_frame'
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='duro_gps_imu_tf_publisher',
            output='screen',
            arguments=[
                '--x',  '0.0',
                '--y',  '0.0',
                '--z',  '0.2',
                '--qx', '0.0',
                '--qy', '0.0',
                '--qz', '0.0',
                '--qw', '1.0',

                '--frame-id',       ns_vehicle + '/' + 'base_link',
                '--child-frame-id', ns_vehicle + '/' + 'duro_gps_imu'
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_ground_link_publisher',
            output='screen',
            arguments=[
                '--x',  '0.0',
                '--y',  '0.0',
                '--z',  '-0.37',
                '--qx', '0.0',
                '--qy', '0.0',
                '--qz', '0.0',
                '--qw', '1.0',
                
                '--frame-id',       ns_vehicle + '/' + 'base_link',
                '--child-frame-id', ns_vehicle + '/' + 'ground_link'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='left1_os_front_tf_publisher',
            output='screen',
            arguments=[
                '--x',     '0.75',
                '--y',     '0.5',
                '--z',     '1.3',
                '--yaw',   '0.0',
                '--pitch', '0.0',
                '--roll',  '0.0',

                '--frame-id',       ns_vehicle + '/' + 'base_link',
                '--child-frame-id', ns_vehicle + '/' + 'os_left_a'
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='right1_os_front_tf_publisher',
            output='screen',
            arguments=[
                '--x',     '1.53',
                '--y',     '-0.5',
                '--z',     '1.41',
                '--yaw',   '0.0',
                '--pitch', '0.0',
                '--roll',  '0.0',

                '--frame-id',       ns_vehicle + '/' + 'base_link',
                '--child-frame-id', ns_vehicle + '/' + 'os_right_a'
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='center1_os_front_tf_publisher',
            output='screen',
            arguments=[
                '--x',     '0.75',
                '--y',     '0.0',
                '--z',     '1.91',
                '--yaw',   '0.0',
                '--pitch', '0.0',
                '--roll',  '0.0',

                '--frame-id',       ns_vehicle + '/' + 'base_link',
                '--child-frame-id', ns_vehicle + '/' + 'os_center_a'
            ],
        )
    ])
