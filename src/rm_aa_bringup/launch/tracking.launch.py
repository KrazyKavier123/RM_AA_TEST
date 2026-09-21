from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def validate_hardware_mode(context):
    enabled = LaunchConfiguration('hardware').perform(context).lower() in ('true', '1')
    mocked = LaunchConfiguration('mock_gimbal').perform(context).lower() in ('true', '1')
    simulated = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1')
    if enabled and (mocked or simulated):
        raise RuntimeError('hardware requires mock_gimbal:=false and use_sim_time:=false')
    return []


def generate_launch_description():
    bringup = Path(get_package_share_directory('rm_aa_bringup'))
    hardware = Path(get_package_share_directory('rm_aa_hardware'))
    return LaunchDescription([
        DeclareLaunchArgument('hardware', default_value='false'),
        DeclareLaunchArgument('hardware_config', default_value=str(bringup / 'config/hardware.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('mock_gimbal', default_value='false',
                             description='Fixed camera TF and zero joint feedback for interface demo only'),
        OpaqueFunction(function=validate_hardware_mode),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup / 'launch/perception.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup / 'launch/control.launch.py'))),
        Node(package='rm_aa_control', executable='mock_gimbal.py', output='screen',
             condition=IfCondition(LaunchConfiguration('mock_gimbal'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(hardware / 'launch/hardware.launch.py')),
                                 condition=IfCondition(LaunchConfiguration('hardware')),
                                 launch_arguments={'hardware_config': LaunchConfiguration('hardware_config')}.items()),
    ])
