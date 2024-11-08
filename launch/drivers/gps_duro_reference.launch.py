from launch import LaunchDescription
from launch_ros.actions import Node
"""
orientation_source can be gps / odom  
- gps: orientation provided from the default gps modules 
- odom: orientation counted from previous positions        
z_coord_ref_switch can be zero / exact / zero_based / orig 
- zero: the Z coordinate is always 0
- exact: the Z coorindinate is always z_coord_exact_height param (must be set in this launch)
- zero_based: Z coordinate starts from 0 and relative
- orig: the original Z provided by Duro / Piksi
euler_based_orientation:
- true: euler based, not enabled by default, please enable SPB message SBP_MSG_ORIENT_EULER 0x0221 decimal 545
- false: quaternion based, not enabled by default, please enable SPB message SBP_MSG_ORIENT_QUAT 0x0220 decimal 544
"""

# -697237.0 -5285644.0 map_gyor_0
# -639770.0 -5195040.0 map_zala_0

def generate_launch_description():
    ns_vehicle = "lexus3"
    node_id = "/gps/duro"
    ld = LaunchDescription()
    duro_node = Node(
        package="duro_gps_driver",
        executable="duro_node",
        parameters=[
            {"ip_address": "192.168.10.11"}, 
            {"port": 55555},
            {"gps_receiver_frame_id": ns_vehicle + "duro"},
            {"imu_frame_id": ns_vehicle + "duro"},
            {"utm_frame_id": "map"},
            {"orientation_source": "gps"},
            {"z_coord_ref_switch": "exact"},
            # {"x_coord_offset": 0.0},
            # {"y_coord_offset": 0.0},
            # {"x_coord_offset": -697237.0}, # map_gyor_0
            # {"y_coord_offset": -5285644.0}, # map_gyor_0
            # {"x_coord_offset": -639770.0}, # map_zala_0
            # {"y_coord_offset": -5195040.0}, # map_zala_0
            {"z_coord_exact_height": 1.8},
            {"tf_frame_id": "map"},
            {"zero_based_pose": False},
            {"tf_child_frame_id": "lexus3/gps"},
            {"euler_based_orientation": True}           
        ],
        namespace=ns_vehicle + node_id,
    )
    ld.add_action(duro_node)
    return ld