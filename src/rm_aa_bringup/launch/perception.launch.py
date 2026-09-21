from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def start_perception(context):
    overrides = {
        'source.mode': ParameterValue(LaunchConfiguration('source').perform(context), value_type=str),
        'source.device': ParameterValue(LaunchConfiguration('device').perform(context), value_type=str),
    }
    video = LaunchConfiguration('video').perform(context)
    if video:
        overrides['source.video_path'] = ParameterValue(video, value_type=str)
    return [Node(package='rm_aa_vision', executable='perception_node', name='perception',
                 output='screen', parameters=[LaunchConfiguration('config'), overrides])]


def generate_launch_description():
    share = Path(get_package_share_directory('rm_aa_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share / 'config' / 'perception.yaml')),
        DeclareLaunchArgument('source', default_value='video', choices=['camera', 'video', 'topic']),
        DeclareLaunchArgument('device', default_value='0'),
        DeclareLaunchArgument('video', default_value=''),
        DeclareLaunchArgument('web', default_value='true'),
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='8080'),
        OpaqueFunction(function=start_perception),
        Node(package='rm_aa_webui', executable='server.py',
             output='screen', condition=IfCondition(LaunchConfiguration('web')),
             arguments=['--ros', '--host', LaunchConfiguration('host'), '--port', LaunchConfiguration('port')]),
    ])
