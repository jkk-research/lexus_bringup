import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    twist_topic_arg = DeclareLaunchArgument(
        'twist_topic',
        default_value=['/sensing/', LaunchConfiguration('kinematic_state_source'), '/twist'],
        description='Name of the twist topic. Default: derived from kinemtaic_state_source arg.'
    )
    accel_topic_arg = DeclareLaunchArgument(
        'accel_topic',
        default_value=['/sensing/', LaunchConfiguration('kinematic_state_source'), '/accel'],
        description='Name of the accel topic. Default: derived from kinemtaic_state_source arg.'
    )

    # radar doppler compensation
    doppcomp_input_pcd_topic_arg = DeclareLaunchArgument(
        'doppcomp/input_pcd_topic',
        default_value='points',
        description='Input PointCloud2 topic for the doppler compensator.'
    )
    doppcomp_output_pcd_topic_arg = DeclareLaunchArgument(
        'doppcomp/output_pcd_topic',
        default_value='dvcompensated_points',
        description='Input PointCloud2 topic for the doppler compensator.'
    )
    doppcomp_override_ego_twist_frame_arg = DeclareLaunchArgument(
        'doppcomp/override_ego_twist_frame',
        default_value='base_link', # TODO add frame_id to twist topic then remove this
        description='Override the frame of the radar for the transformation of the points. Leave empty to use the frameid defined in the input point cloud message.'
    )

    # radar pointcloud aggregation
    pcdagg_input_pcd_topic_arg = DeclareLaunchArgument(
        'pcdagg/input_pcd_topic',
        default_value='dvcompensated_points',
        description='Input PointCloud2 topic for the aggregator.'
    )
    pcdagg_output_pcd_topic_arg = DeclareLaunchArgument(
        'pcdagg/output_pcd_topic',
        default_value='aggregated_points',
        description='Output PointCloud2 topic for the aggregator.'
    )
    pcdagg_aggregation_time_window_arg = DeclareLaunchArgument(
        'pcdagg/aggregation_time_window_sec',
        default_value='0.5',
        description='Time window for the aggregation.'
    )
    pcdagg_ego_twist_frame_arg = DeclareLaunchArgument(
        'pcdagg/override_ego_twist_frame',
        default_value='base_link', # TODO add frame_id to twist topic then remove this
        description='Override the frame of the radar for the transformation of the points. Leave empty to use the frameid defined in the input point cloud message.'
    )

    # radar pointcloud merger
    radar_pointcloud_merger_in_pcd_topics_arg = DeclareLaunchArgument(
        'pcdmerger/in_pcd_topics',
        default_value='''[
            /sensing/radar/fc/aggregated_points, 
            /sensing/radar/fl/aggregated_points, 
            /sensing/radar/fr/aggregated_points, 
            /sensing/radar/rl/aggregated_points, 
            /sensing/radar/rr/aggregated_points
        ]''',
        description='PointCloud2 topics to be merged. The first one will be the trigger for merging.'
    )
    radar_pointcloud_merger_out_topic_arg = DeclareLaunchArgument(
        'pcdmerger/out_topic',
        default_value='merged_points',
        description='Output PointCloud2 topic for the merger.'
    )
    radar_pointcloud_merger_ego_frame_arg = DeclareLaunchArgument(
        'pcdmerger/ego_frame',
        default_value='base_link',
        description='Input localization (PoseWithCovarianceStamped) topic for the aggregator.'
    )
    radar_pointcloud_merger_merge_trigger_timeout_arg = DeclareLaunchArgument(
        'pcdmerger/merge_trigger_timeout_sec',
        default_value='0.1',
        description='Timeout for the trigger topic. The trigger is changed if the current trigger times out.'
    )

    radar_doppler_compensator_fc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(
                get_package_share_directory('radar_pointcloud_preprocessor'),
                'launch',
                'dopplerCompensator.launch.py'
            )
        ),
        launch_arguments={
            'doppcomp/namespace'               : '/sensing/radar/fc',
            'doppcomp/input_pcd_topic'         : LaunchConfiguration('doppcomp/input_pcd_topic'),
            'doppcomp/output_pcd_topic'        : LaunchConfiguration('doppcomp/output_pcd_topic'),
            'doppcomp/twist_topic'             : LaunchConfiguration('twist_topic'),
            'doppcomp/override_ego_twist_frame': LaunchConfiguration('doppcomp/override_ego_twist_frame')
        }.items()
    )

    radar_pointcloud_aggregator_fc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(
                get_package_share_directory('radar_pointcloud_preprocessor'),
                'launch',
                'pointcloudAggregator.launch.py'
            )
        ),
        launch_arguments={
            'pcdagg/namespace'                  : '/sensing/radar/fc',
            'pcdagg/input_pcd_topic'            : LaunchConfiguration('pcdagg/input_pcd_topic'),
            'pcdagg/output_pcd_topic'           : LaunchConfiguration('pcdagg/output_pcd_topic'),
            'pcdagg/twist_topic'                : LaunchConfiguration('twist_topic'),
            'pcdagg/aggregation_time_window_sec': LaunchConfiguration('pcdagg/aggregation_time_window_sec'),
            'pcdagg/override_ego_twist_frame'   : LaunchConfiguration('pcdagg/override_ego_twist_frame')
        }.items()
    )

    radar_doppler_compensator_fl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(
                get_package_share_directory('radar_pointcloud_preprocessor'),
                'launch',
                'dopplerCompensator.launch.py'
            )
        ),
        launch_arguments={
            'doppcomp/namespace'               : '/sensing/radar/fl',
            'doppcomp/input_pcd_topic'         : LaunchConfiguration('doppcomp/input_pcd_topic'),
            'doppcomp/output_pcd_topic'        : LaunchConfiguration('doppcomp/output_pcd_topic'),
            'doppcomp/twist_topic'             : LaunchConfiguration('twist_topic'),
            'doppcomp/override_ego_twist_frame': LaunchConfiguration('doppcomp/override_ego_twist_frame')
        }.items()
    )

    radar_pointcloud_aggregator_fl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(
                get_package_share_directory('radar_pointcloud_preprocessor'),
                'launch',
                'pointcloudAggregator.launch.py'
            )
        ),
        launch_arguments={
            'pcdagg/namespace'                  : '/sensing/radar/fl',
            'pcdagg/input_pcd_topic'            : LaunchConfiguration('pcdagg/input_pcd_topic'),
            'pcdagg/output_pcd_topic'           : LaunchConfiguration('pcdagg/output_pcd_topic'),
            'pcdagg/twist_topic'                : LaunchConfiguration('twist_topic'),
            'pcdagg/aggregation_time_window_sec': LaunchConfiguration('pcdagg/aggregation_time_window_sec'),
            'pcdagg/override_ego_twist_frame'   : LaunchConfiguration('pcdagg/override_ego_twist_frame')
        }.items()
    )

    radar_doppler_compensator_fr = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(
                get_package_share_directory('radar_pointcloud_preprocessor'),
                'launch',
                'dopplerCompensator.launch.py'
            )
        ),
        launch_arguments={
            'doppcomp/namespace'               : '/sensing/radar/fr',
            'doppcomp/input_pcd_topic'         : LaunchConfiguration('doppcomp/input_pcd_topic'),
            'doppcomp/output_pcd_topic'        : LaunchConfiguration('doppcomp/output_pcd_topic'),
            'doppcomp/twist_topic'             : LaunchConfiguration('twist_topic'),
            'doppcomp/override_ego_twist_frame': LaunchConfiguration('doppcomp/override_ego_twist_frame')
        }.items()
    )

    radar_pointcloud_aggregator_fr = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(
                get_package_share_directory('radar_pointcloud_preprocessor'),
                'launch',
                'pointcloudAggregator.launch.py'
            )
        ),
        launch_arguments={
            'pcdagg/namespace'                  : '/sensing/radar/fr',
            'pcdagg/input_pcd_topic'            : LaunchConfiguration('pcdagg/input_pcd_topic'),
            'pcdagg/output_pcd_topic'           : LaunchConfiguration('pcdagg/output_pcd_topic'),
            'pcdagg/twist_topic'                : LaunchConfiguration('twist_topic'),
            'pcdagg/aggregation_time_window_sec': LaunchConfiguration('pcdagg/aggregation_time_window_sec'),
            'pcdagg/override_ego_twist_frame'   : LaunchConfiguration('pcdagg/override_ego_twist_frame')
        }.items()
    )

    radar_doppler_compensator_rl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(
                get_package_share_directory('radar_pointcloud_preprocessor'),
                'launch',
                'dopplerCompensator.launch.py'
            )
        ),
        launch_arguments={
            'doppcomp/namespace'               : '/sensing/radar/rl',
            'doppcomp/input_pcd_topic'         : LaunchConfiguration('doppcomp/input_pcd_topic'),
            'doppcomp/output_pcd_topic'        : LaunchConfiguration('doppcomp/output_pcd_topic'),
            'doppcomp/twist_topic'             : LaunchConfiguration('twist_topic'),
            'doppcomp/override_ego_twist_frame': LaunchConfiguration('doppcomp/override_ego_twist_frame')
        }.items()
    )

    radar_pointcloud_aggregator_rl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(
                get_package_share_directory('radar_pointcloud_preprocessor'),
                'launch',
                'pointcloudAggregator.launch.py'
            )
        ),
        launch_arguments={
            'pcdagg/namespace'                  : '/sensing/radar/rl',
            'pcdagg/input_pcd_topic'            : LaunchConfiguration('pcdagg/input_pcd_topic'),
            'pcdagg/output_pcd_topic'           : LaunchConfiguration('pcdagg/output_pcd_topic'),
            'pcdagg/twist_topic'                : LaunchConfiguration('twist_topic'),
            'pcdagg/aggregation_time_window_sec': LaunchConfiguration('pcdagg/aggregation_time_window_sec'),
            'pcdagg/override_ego_twist_frame'   : LaunchConfiguration('pcdagg/override_ego_twist_frame')
        }.items()
    )

    radar_doppler_compensator_rr = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(
                get_package_share_directory('radar_pointcloud_preprocessor'),
                'launch',
                'dopplerCompensator.launch.py'
            )
        ),
        launch_arguments={
            'doppcomp/namespace'               : '/sensing/radar/rr',
            'doppcomp/input_pcd_topic'         : LaunchConfiguration('doppcomp/input_pcd_topic'),
            'doppcomp/output_pcd_topic'        : LaunchConfiguration('doppcomp/output_pcd_topic'),
            'doppcomp/twist_topic'             : LaunchConfiguration('twist_topic'),
            'doppcomp/override_ego_twist_frame': LaunchConfiguration('doppcomp/override_ego_twist_frame')
        }.items()
    )

    radar_pointcloud_aggregator_rr = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(
                get_package_share_directory('radar_pointcloud_preprocessor'),
                'launch',
                'pointcloudAggregator.launch.py'
            )
        ),
        launch_arguments={
            'pcdagg/namespace'                  : '/sensing/radar/rr',
            'pcdagg/input_pcd_topic'            : LaunchConfiguration('pcdagg/input_pcd_topic'),
            'pcdagg/output_pcd_topic'           : LaunchConfiguration('pcdagg/output_pcd_topic'),
            'pcdagg/twist_topic'                : LaunchConfiguration('twist_topic'),
            'pcdagg/aggregation_time_window_sec': LaunchConfiguration('pcdagg/aggregation_time_window_sec'),
            'pcdagg/override_ego_twist_frame'   : LaunchConfiguration('pcdagg/override_ego_twist_frame')
        }.items()
    )

    radar_pointcloud_merger = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(
                get_package_share_directory('radar_pointcloud_preprocessor'),
                'launch',
                'pointcloudMerger.launch.py'
            )
        ),
        launch_arguments={
            'pcdmerger/namespace'                : '/sensing/radar',
            'pcdmerger/in_pcd_topics'            : LaunchConfiguration('pcdmerger/in_pcd_topics'),
            'pcdmerger/out_topic'                : LaunchConfiguration('pcdmerger/out_topic'),
            'pcdmerger/ego_frame'                : LaunchConfiguration('pcdmerger/ego_frame'),
            'pcdmerger/merge_trigger_timeout_sec': LaunchConfiguration('pcdmerger/merge_trigger_timeout_sec'),
        }.items()
    )

    return LaunchDescription([

        twist_topic_arg,
        accel_topic_arg,
        doppcomp_input_pcd_topic_arg,
        doppcomp_output_pcd_topic_arg,
        doppcomp_override_ego_twist_frame_arg,
        pcdagg_input_pcd_topic_arg,
        pcdagg_output_pcd_topic_arg,
        pcdagg_aggregation_time_window_arg,
        pcdagg_ego_twist_frame_arg,
        radar_doppler_compensator_fc,
        radar_pointcloud_aggregator_fc,
        radar_doppler_compensator_fl,
        radar_pointcloud_aggregator_fl,
        radar_doppler_compensator_fr,
        radar_pointcloud_aggregator_fr,
        radar_doppler_compensator_rl,
        radar_pointcloud_aggregator_rl,
        radar_doppler_compensator_rr,
        radar_pointcloud_aggregator_rr,
        radar_pointcloud_merger,
    ])





