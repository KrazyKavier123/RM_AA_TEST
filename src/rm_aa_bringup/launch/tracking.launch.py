from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = Path(get_package_share_directory('rm_aa_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('mock_gimbal', default_value='false',
                             description='Fixed camera TF and zero joint feedback for interface demo only'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup / 'launch/perception.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup / 'launch/control.launch.py'))),
        Node(package='rm_aa_control', executable='mock_gimbal.py', output='screen',
             condition=IfCondition(LaunchConfiguration('mock_gimbal'))),
    ])
