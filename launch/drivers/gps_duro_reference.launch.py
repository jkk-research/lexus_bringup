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
def generate_launch_description():
    namespace_lx = "lexus3"
    node_id = "/gps/duro"
    ld = LaunchDescription()
    duro_node = Node(
        package="duro_gps_driver",
        executable="duro_node",
        parameters=[
            {"ip_address": "192.168.10.10"},
            {"port": 55555},
            {"gps_receiver_frame_id": namespace_lx + "duro"},
            {"imu_frame_id": namespace_lx + "duro"},
            {"utm_frame_id": "map"},
            {"orientation_source": "gps"},
            {"z_coord_ref_switch": "exact"},
            {"z_coord_exact_height": 1.8},
            {"tf_frame_id": "map"},
            {"zero_based_pose": False},
            {"tf_child_frame_id": "lexus3/gps"},
            {"euler_based_orientation": True}           
        ],
        namespace=namespace_lx + node_id,
    )
    ld.add_action(duro_node)
    return ld