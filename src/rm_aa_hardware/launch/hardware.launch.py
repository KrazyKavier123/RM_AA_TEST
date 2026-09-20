from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('rm_aa_hardware'))
    return LaunchDescription([
        DeclareLaunchArgument('hardware_config', default_value=str(share / 'config/hardware.yaml')),
        Node(package='rm_aa_hardware', executable='hardware_node', output='screen',
             parameters=[LaunchConfiguration('hardware_config')]),
    ])
