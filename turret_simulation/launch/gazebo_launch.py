import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Paths
    pkg_desc = get_package_share_directory('turret_description')
    pkg_sim = get_package_share_directory('turret_simulation')
    xacro_file = os.path.join(pkg_desc, 'urdf', 'turretV2.urdf.xacro')
    robot_desc_raw = xacro.process_file(xacro_file).toxml()
    world_file = os.path.join(pkg_sim, 'world', 'arena.world')
    

    gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    launch_arguments={
        'world': world_file,
        'extra_gazebo_args': '--verbose' # This helps you see why plugins fail
    }.items()
    )

    node_robot_state_publisher = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output = 'screen',
        parameters = [{'robot_description': robot_desc_raw, 'use_sim_time': True}]
    )

    node_spawn_entity = Node(
        package = 'gazebo_ros',
        executable = 'spawn_entity.py',
        arguments = ['-topic', 'robot_description', '-entity', 'my_turret'],
        output = 'screen'
    )

    # joint_state_broadcaster_spawner = Node(
    # package="controller_manager",
    # executable="spawner",
    # arguments=["joint_state_broadcaster"],
    # )

    # # 2. Spawner for your Turret Controller
    # turret_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["turret_controller"],
    # )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        node_spawn_entity,
        # joint_state_broadcaster_spawner,
        # turret_controller_spawner
    ])