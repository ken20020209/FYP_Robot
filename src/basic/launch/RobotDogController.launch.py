import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    bringup_launch_path = PathJoinSubstitution(
        [FindPackageShare('basic'), 'bringup.launch.py']
    )
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch_path)
        ),
        Node(
            package='basic',
            namespace='',
            executable='Camera'
        ),
        Node(
            package='basic',
            namespace='',
            executable='Action'
        ),
    ])