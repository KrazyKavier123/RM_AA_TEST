from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware_config',
                             description='Required path to hardware parameter YAML; bringup supplies its config/hardware.yaml'),
        Node(package='rm_aa_hardware', executable='hardware_node', output='screen',
             parameters=[LaunchConfiguration('hardware_config')]),
    ])
