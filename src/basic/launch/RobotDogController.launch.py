import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node,PushRosNamespace

def generate_launch_description():
    bringup_launch_path = PathJoinSubstitution(
        [FindPackageShare('basic'), 'bringup.launch.py']
    )

    namespace=LaunchConfiguration('namespace',default='RobotDogConnector')
    namespace_declare=DeclareLaunchArgument(
            'namespace',
            default_value=namespace,
            description='Name of the RobotDogConnector node'
        )
    

    group = GroupAction([
        PushRosNamespace(namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch_path)
        ),
        Node(
            package='basic',
            executable='Camera'
        ),
        Node(
            package='basic',
            executable='Action'
        )
    ])

    ld = LaunchDescription()

    ld.add_action(namespace_declare)
    ld.add_action(group)

    return ld