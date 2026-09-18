from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory('rm_aa_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share / 'config' / 'perception.yaml')),
        DeclareLaunchArgument('source', default_value='synthetic'),
        DeclareLaunchArgument('device', default_value='0'),
        DeclareLaunchArgument('video', default_value=''),
        DeclareLaunchArgument('web', default_value='true'),
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='8080'),
        Node(package='rm_aa_vision', executable='perception_node', name='perception',
             output='screen', parameters=[LaunchConfiguration('config'), {
                 'source.mode': ParameterValue(LaunchConfiguration('source'), value_type=str),
                 'source.device': ParameterValue(LaunchConfiguration('device'), value_type=str),
                 'source.video_path': ParameterValue(LaunchConfiguration('video'), value_type=str),
             }]),
        Node(package='rm_aa_bringup', executable='debug_web.py', name='debug_web',
             output='screen', condition=IfCondition(LaunchConfiguration('web')),
             arguments=['--host', LaunchConfiguration('host'), '--port', LaunchConfiguration('port'),
                        '--web-root', str(share / 'web')]),
    ])
