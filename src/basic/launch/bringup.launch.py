from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    laser = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("oradar_lidar"),
                "launch",
                "ms200_scan.launch.py",
            )),
        )

    
    yahboom_dog_joint_state = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("yahboom_dog_joint_state"),
                "",
                "yahboom_dog_joint_state.launch.py",
            )),
        )


    xgo_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("basic"),
                "",
                "xgo_urdf.launch.py",
            )),
        )

    
    yahboom_base = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("yahboom_base"),
                "",
                "yahboom_base.launch.py",
            )),
        )

    tf_basefootprint = Node(
        package='tf_publisher',
        node_executable="tf_publisher",
        output="screen"
    )


    app_map_save_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("yahboom_app_save_map"),
                "",
                "yahboom_app_save_map.launch.py",
            )),
    )

    return LaunchDescription(
        [
            laser,
            yahboom_dog_joint_state,
            xgo_description,
            yahboom_base,
            tf_basefootprint
        ]
    )