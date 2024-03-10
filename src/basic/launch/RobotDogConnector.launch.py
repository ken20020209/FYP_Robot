from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'type',
            default_value='dog_s2',
            description='Type of the Robot'
        ),
        DeclareLaunchArgument(
            'name',
            default_value='RobotDogConnector',
            description='Name of the RobotDogConnector node'
        ),
        Node(
            package='basic',
            executable='Oled',
        ),
        Node(
            package='basic',
            namespace=LaunchConfiguration('name'),
            executable='RobotDogConnector',
            parameters=[{'type': LaunchConfiguration('type')},
                        {'name': LaunchConfiguration('name')},]

        )
    ])