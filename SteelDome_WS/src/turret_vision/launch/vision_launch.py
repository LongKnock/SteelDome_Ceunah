import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():  
    Node(
        package='turret_vision',
        executable='node_vision',
        name='vision_node',
        output='screen',
        parameters=[
            {'backend': 'cpu'},
            {'use_sim_time': True}
        ],

        remappings=[
            ('/image_raw', '/my_bot/camera1/image_raw')            
        ],
    )
