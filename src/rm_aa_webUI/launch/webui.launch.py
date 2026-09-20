from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='8080'),
        Node(package='rm_aa_webui', executable='server.py', output='screen',
             arguments=['--ros', '--host', LaunchConfiguration('host'), '--port', LaunchConfiguration('port')]),
    ])
