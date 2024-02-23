#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription

'''
parameters=[
        {'device_model': 'MS200'},
        {'frame_id': 'laser_frame'},
        {'scan_topic': '/scan'},
        {'port_name': '/dev/ttyAMA1'},
        {'baudrate': 230400},
        {'angle_min': 0.0},
        {'angle_max': 360.0},
        {'range_min': 0.05},
        {'range_max': 20.0},
        {'clockwise': False},
        {'motor_speed': 10}
      ]
'''

def generate_launch_description():
  namespace=LaunchConfiguration('namespace',default='RobotDogConnector')
  namespace_declare=DeclareLaunchArgument(
            'namespace',
            default_value=namespace,
            description='Name of the RobotDogConnector node'
        )
  # LiDAR publisher node
  ordlidar_node = Node(
      package='oradar_lidar',
      node_executable='oradar_scan',
      node_name='MS200',
      output='screen',
      parameters=[
        {'device_model': 'MS200'},
        {'frame_id': PythonExpression(["'",namespace,"'+","'/laser_frame'"])},
        {'scan_topic': 'scan'},
        {'port_name': '/dev/ttyAMA1'},
        {'baudrate': 230400},
        {'angle_min': 0.0},
        {'angle_max': 360.0},
        {'range_min': 0.05},
        {'range_max': 20.0},
        {'clockwise': False},
        {'motor_speed': 10}
      ]
  )

  # base_link to laser_frame tf node
  base_link_to_laser_tf_node = Node(
    package='tf2_ros',
    node_executable='static_transform_publisher',
    node_name='base_link_to_base_laser',
    arguments=['0','0','0.18','0','0','0',PythonExpression(["'",namespace,"'+","'/base_link'"]),PythonExpression(["'",namespace,"'+","'/laser_frame'"])]
  )


  # Define LaunchDescription variable
  ord = LaunchDescription()

  ord.add_action(namespace_declare)
  ord.add_action(ordlidar_node)
  ord.add_action(base_link_to_laser_tf_node)

  return ord
