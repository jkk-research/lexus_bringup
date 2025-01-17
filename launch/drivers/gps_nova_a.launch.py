



from launch import LaunchDescription
import launch_ros.actions

nova_pkg = "novatel_gps_driver"
ns_vehicle = "lexus3"
ns_nova = "/gps/nova"

# -697237.0 -5285644.0 map_gyor_0
# -639770.0 -5195040.0 map_zala_0

def generate_launch_description():
    container = launch_ros.actions.ComposableNodeContainer(
        name='novatel_gps_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            launch_ros.descriptions.ComposableNode(
                package=nova_pkg,
                namespace= ns_vehicle + ns_nova,
                plugin='novatel_gps_driver::NovatelGpsNode',
                name='novatel_gps',
                parameters=[{
                    'connection_type': 'tcp',
                    'device': '192.168.10.12:3001',
                    'verbose': False,
                    'imu_sample_rate': -1.0,
                    'use_binary_messages': True,
                    'publish_novatel_positions': False,
                    'publish_imu_messages_': True,
                    'publish_novatel_utm_positions': True,
                    'publish_imu_messages': True,
                    'publish_novatel_velocity': False,
                    'publish_novatel_psrdop2': False,
                    'imu_frame_id': '/lexus3/nova/imu',
                    'frame_id': ns_vehicle + ns_nova,
                    'publish_novatel_dual_antenna_heading': True,
                    # 'x_coord_offset': 0.0,
                    # 'y_coord_offset': 0.0,
                    # 'x_coord_offset': -697237.0, # map_gyor_0
                    # 'y_coord_offset': -5285644.0, # map_gyor_0
                    # 'x_coord_offset': -639770.0, # map_zala_0
                    # 'y_coord_offset': -5195040.0, # map_zala_0
                    'z_coord_exact_height': 1.8,
                    'z_coord_ref_switch': "exact",
                    'tf_frame_id': "map",
                    'tf_child_frame_id': ns_vehicle + "/gps",
                    'utm_frame_id': "map",
                }]
            )
        ],
        output='screen'
    )

    return LaunchDescription([container])
