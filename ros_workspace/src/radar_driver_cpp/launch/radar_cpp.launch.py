from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    radar_config_filename = LaunchConfiguration('radar_config_filename')
    dca_config_filename = LaunchConfiguration('dca_config_filename')
    enable_processing = LaunchConfiguration('enable_processing')
    rd_every_n_frames = LaunchConfiguration('rd_every_n_frames')
    pointcloud_every_n_frames = LaunchConfiguration('pointcloud_every_n_frames')

    radar_cpp_publisher_node = Node(
        package='radar_driver_cpp',
        executable='publisher',
        name='radar_publisher_cpp',
        output='screen',
        parameters=[{
            'radar_config_filename': radar_config_filename,
            'dca_config_filename': dca_config_filename,
        }]
    )

    radar_cpp_subscriber_node = Node(
        package='radar_driver_cpp',
        executable='subscriber',
        name='radar_subscriber_cpp',
        output='screen',
        prefix='nice -n 10',
        parameters=[{
            'radar_config_filename': radar_config_filename,
            'rd_every_n_frames': rd_every_n_frames,
            'pointcloud_every_n_frames': pointcloud_every_n_frames,
        }],
        condition=IfCondition(enable_processing)
    )

    return LaunchDescription([
        DeclareLaunchArgument('radar_config_filename', default_value='AWR2243_mmwaveconfig_max15.txt'),
        DeclareLaunchArgument('dca_config_filename', default_value='dca_config.txt'),
        DeclareLaunchArgument('enable_processing', default_value='false'),
        DeclareLaunchArgument('rd_every_n_frames', default_value='1'),
        DeclareLaunchArgument('pointcloud_every_n_frames', default_value='20'),
        radar_cpp_publisher_node,
        radar_cpp_subscriber_node
    ])
