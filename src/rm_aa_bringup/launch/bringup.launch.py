"""Start the project: perception, tracking/control and optional web/mock nodes."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('rm_aa_bringup'))
    arguments = [
        DeclareLaunchArgument('config', default_value=str(share / 'config/perception.yaml'),
                             description='Perception parameter YAML'),
        DeclareLaunchArgument('control_config', default_value=str(share / 'config/control.yaml'),
                             description='Tracking and control parameter YAML'),
        DeclareLaunchArgument('source', default_value='video',
                             choices=['camera', 'video', 'topic']),
        DeclareLaunchArgument('device', default_value='0'),
        DeclareLaunchArgument('video', default_value='', description='Video directory or file; empty uses YAML source.video_path'),
        DeclareLaunchArgument('web', default_value='true'),
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('mock_gimbal', default_value='false',
                             description='Provide fixed TF and zero feedback for interface demos'),
        DeclareLaunchArgument('hardware', default_value='false',
                             description='Start the 64-byte gimbal serial bridge; incompatible with mock_gimbal'),
        DeclareLaunchArgument('hardware_config', default_value=str(share / 'config/hardware.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                             description='Use /clock for the control node; external sources must use matching time'),
    ]
    names = ('config', 'control_config', 'source', 'device', 'video',
             'web', 'host', 'port', 'mock_gimbal', 'use_sim_time', 'hardware', 'hardware_config')
    return LaunchDescription(arguments + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/tracking.launch.py')),
            launch_arguments={name: LaunchConfiguration(name) for name in names}.items(),
        ),
    ])
