from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'lvx_file_path',
            description='Absolute path to the LVX file. This argument is required.'
            # No default_value, so it's a required command-line argument
        ),
        DeclareLaunchArgument(
            'point_cloud_topic_prefix',
            default_value='livox/lidar_',
            description='Prefix for PointCloud2 topics for each lidar.'
        ),
        DeclareLaunchArgument(
            'imu_topic_prefix',
            default_value='livox/imu_',
            description='Prefix for Imu topics for each IMU data stream.'
        ),
        DeclareLaunchArgument(
            'base_frame_id',
            default_value='livox_base',
            description='Base frame_id for TF transforms and message headers.'
        ),
        DeclareLaunchArgument(
            'lidar_frame_id_prefix',
            default_value='livox_lidar_',
            description='Prefix for individual lidar frame_ids (e.g., livox_lidar_000000000000001).'
        ),
        DeclareLaunchArgument(
            'use_original_timestamps',
            default_value='True',
            description='Use original timestamps from LVX file if True, otherwise use current ROS time. (bool)'
        ),
        DeclareLaunchArgument(
            'playback_rate_hz',
            default_value='20.0',
            description='Rate (in Hz) at which to publish messages. (double)'
        ),
        DeclareLaunchArgument(
            'loop_playback',
            default_value='False',
            description='Loop LVX file playback if True. (bool)'
        ),

        Node(
            package='livox_lvx2_ros2_publisher',
            executable='lvx2_publisher',  # Entry point from setup.py
            name='lvx2_parser_node',      # Node name
            output='screen',
            parameters=[{
                'lvx_file_path': LaunchConfiguration('lvx_file_path'),
                'point_cloud_topic_prefix': LaunchConfiguration('point_cloud_topic_prefix'),
                'imu_topic_prefix': LaunchConfiguration('imu_topic_prefix'),
                'base_frame_id': LaunchConfiguration('base_frame_id'),
                'lidar_frame_id_prefix': LaunchConfiguration('lidar_frame_id_prefix'),
                'use_original_timestamps': LaunchConfiguration('use_original_timestamps'),
                'playback_rate_hz': LaunchConfiguration('playback_rate_hz'),
                'loop_playback': LaunchConfiguration('loop_playback'),
            }]
        )
    ])
