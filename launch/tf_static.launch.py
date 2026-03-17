from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'tf_static_ns',
            default_value='lexus3',
            description='Namespace for static transforms'),

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

                '--frame-id',       [LaunchConfiguration('tf_static_ns'), '/map'],
                '--child-frame-id', [LaunchConfiguration('tf_static_ns'), '/map_gyor_0']
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

                '--frame-id',       [LaunchConfiguration('tf_static_ns'), '/map'],
                '--child-frame-id', [LaunchConfiguration('tf_static_ns'), '/map_zala_0']
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

                '--frame-id',       [LaunchConfiguration('tf_static_ns'), '/base_link'],
                '--child-frame-id', [LaunchConfiguration('tf_static_ns'), '/duro_gps']
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_gps_tf_publisher',
            output='screen',
            # https://raw.githubusercontent.com/wiki/szenergy/szenergy-public-resources/img/2022.L.01.svg
            # TODO
            arguments=[
                '--x',     '0.0', ## Based on Novatel Application Suite By default, INS position is reported at the centre of the IMU. (Set to IMU)
                '--y',     '0.0',
                '--z',     '-0.2',
                '--yaw',   '0.0',
                '--pitch', '0.0',
                '--roll',  '0.0',

                '--frame-id',       [LaunchConfiguration('tf_static_ns'), '/gps'],
                '--child-frame-id', [LaunchConfiguration('tf_static_ns'), '/base_link']
            ],
        ),
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

                '--frame-id',       [LaunchConfiguration('tf_static_ns'), '/base_link'],
                '--child-frame-id', [LaunchConfiguration('tf_static_ns'), '/zed_camera_link']
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

                '--frame-id',       [LaunchConfiguration('tf_static_ns'), '/base_link'],
                '--child-frame-id', [LaunchConfiguration('tf_static_ns'), '/duro_gps_imu']
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
                
                '--frame-id',       [LaunchConfiguration('tf_static_ns'), '/base_link'],
                '--child-frame-id', [LaunchConfiguration('tf_static_ns'), '/ground_link']
            ],
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

                '--frame-id',       [LaunchConfiguration('tf_static_ns'), '/base_link'],
                '--child-frame-id', [LaunchConfiguration('tf_static_ns'), '/os_left_a']
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

                '--frame-id',       [LaunchConfiguration('tf_static_ns'), '/base_link'],
                '--child-frame-id', [LaunchConfiguration('tf_static_ns'), '/os_right_a']
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

                '--frame-id',       [LaunchConfiguration('tf_static_ns'), '/base_link'],
                '--child-frame-id', [LaunchConfiguration('tf_static_ns'), '/os_center_a']
            ],
        ),Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='center_radar_static_tf',
            output='screen',
            arguments=[
                '--x',     '3.8',
                '--y',     '0.0',
                '--z',     '0.0',
                '--yaw',   '0.0',
                '--pitch', '0.0',
                '--roll',  '0.0',

                '--frame-id',       [LaunchConfiguration('tf_static_ns'), '/base_link'],
                '--child-frame-id', [LaunchConfiguration('tf_static_ns'), '/radar_front_center']
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='front_left_static',
            output='screen',
            arguments=[
                '--x',     '3.5',
                '--y',     '0.835',
                '--z',     '0.0',
                '--yaw',   '1.04719755',
                '--pitch', '0.0',
                '--roll',  '3.141592654',

                '--frame-id',       [LaunchConfiguration('tf_static_ns'), '/base_link'],
                '--child-frame-id', [LaunchConfiguration('tf_static_ns'), '/radar_front_left']
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='front_right_static',
            output='screen',
            arguments=[
                '--x',     '3.5',
                '--y',     '-0.835',
                '--z',     '0.0',
                '--yaw',   '-1.04719755',
                '--pitch', '0.0',
                '--roll',  '0.0',

                '--frame-id',       [LaunchConfiguration('tf_static_ns'), '/base_link'],
                '--child-frame-id', [LaunchConfiguration('tf_static_ns'), '/radar_front_right']
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rear_left_static',
            output='screen',
            arguments=[
                '--x',     '-0.85',
                '--y',     '0.58',
                '--z',     '0.0',
                '--yaw',   '2.0943951',
                '--pitch', '0.0',
                '--roll',  '0.0',

                '--frame-id',       [LaunchConfiguration('tf_static_ns'), '/base_link'],
                '--child-frame-id', [LaunchConfiguration('tf_static_ns'), '/radar_rear_left']
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rear_right_static',
            output='screen',
            arguments=[
                '--x',     '-0.85',
                '--y',     '-0.58',
                '--z',     '0.0',
                '--yaw',   '-2.0943951',
                '--pitch', '0.0',
                '--roll',  '3.141592654',

                '--frame-id',       [LaunchConfiguration('tf_static_ns'), '/base_link'],
                '--child-frame-id', [LaunchConfiguration('tf_static_ns'), '/radar_rear_right']
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='luminar_static',
            output='screen',
            arguments=[
                '--x',     '1.43',
                '--y',     '-0.3',
                '--z',     '1.41',
                '--yaw',   '0.0',
                '--pitch', '0.0',
                '--roll',  '-0.0174533',

                '--frame-id',       [LaunchConfiguration('tf_static_ns'), '/base_link'],
                '--child-frame-id', [LaunchConfiguration('tf_static_ns'), '/luminar_lidar_0']
            ],
        ),
         Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='luminar_tf_publisher',
            output='screen',
            arguments=[
                '--x',     '1.43',
                '--y',     '-0.3',
                '--z',     '1.41',
                '--yaw',   '-0.0174533',
                '--pitch', '0.0',
                '--roll',  '0.0',

                '--frame-id',       ns_vehicle + '/' + 'base_link',
                '--child-frame-id', ns_vehicle + '/' + 'luminar_lidar_0'
            ],
        ),
         Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf_publisher',
            output='screen',
            arguments=[
                '--x',     '0.0',
                '--y',     '0.0',
                '--z',     '0.0',
                '--yaw',   '0.0',
                '--pitch', '0.0',
                '--roll',  '0.0',

                '--frame-id',       ns_vehicle + '/' + 'base_link',
                '--child-frame-id',  'imu_link'
            ],

        ),
    ])
