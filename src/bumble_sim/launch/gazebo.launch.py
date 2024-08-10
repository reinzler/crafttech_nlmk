#!/usr/bin/env python

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    list_of_nodes_to_launch = []

    # Gazebo
    world_file_name = 'my_empty_world.world'
    rover_model = 'r1_rover'
    world = os.path.join(get_package_share_directory('bumble_sim'), 'worlds', world_file_name)
    gazebo_node = ExecuteProcess(cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'], output='screen')

    rover_model_to_load = os.path.join(get_package_share_directory('bumble_sim'), 'models', rover_model, rover_model + ".sdf")

    rover_model_upload = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_robot', '-file', rover_model_to_load],
            output='screen'
            )

    list_of_nodes_to_launch.extend([gazebo_node, rover_model_upload])

    return LaunchDescription(list_of_nodes_to_launch)
