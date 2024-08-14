import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_dir = FindPackageShare('bumble_sim').find('bumble_sim')
    world_file_name = 'my_empty_world.world'
    world = os.path.join(package_dir, 'worlds', world_file_name)

    # Путь к моделям из PX4
    px4_model_path = '/home/vadim/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models'

    # Путь к моделям в текущем ROS2 пакете
    local_model_path = os.path.join(package_dir, 'models')

    # Путь к плагинам PX4
    px4_plugin_path = '/home/vadim/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic'

    # Объединяем пути для GAZEBO_MODEL_PATH
    gazebo_model_path = f"{local_model_path}:{px4_model_path}"

    # Объединяем пути для LD_LIBRARY_PATH
    ld_library_path = f"{os.environ.get('LD_LIBRARY_PATH', '')}:{px4_plugin_path}"

    # Set environment variables for Gazebo model path and plugin path
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path
    )

    set_ld_library_path = SetEnvironmentVariable(
        name='LD_LIBRARY_PATH',
        value=ld_library_path
    )

    # Запуск Gazebo
    gazebo_node = ExecuteProcess(
        cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    spawn_aruco =TimerAction(
        period=10.0,  # Задержка в 2 секунды
        actions=[
            Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'aruco', '-file', os.path.join(package_dir, 'models', 'aruco', 'model.sdf'),
                   '-x', '10', '-y', '10', '-z', '2', '-Y', '1.5708'],
                   # '-x', '0', '-y', '0', '-z', '0', '-Y', '0'],
        output='screen'
        )]
    )

    spawn_rover = TimerAction(
        period=1.0,  # Задержка в 2 секунды
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-entity', 'r1_rover', '-file',
                           os.path.join(package_dir, 'models', 'r1_rover', 'r1_rover.sdf'), '-x', '0', '-y', '0',
                           '-z', '0'],
                output='screen'
            )]
    )

    # Define the nodes to be launched
    return LaunchDescription([
        # set_gazebo_model_path,
        # set_ld_library_path,
        # gazebo_node,
        # spawn_aruco,
        # spawn_rover,

        Node(
            package='bumble_sim',
            namespace='rover_sim',
            executable='processes',
            name='processes',
            prefix='gnome-terminal --',
        ),

        Node(
            package='bumble_sim',
            namespace='rover_sim',
            executable='control',
            name='control',
            prefix='gnome-terminal --',
        ),

        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='bumble_sim',
                    namespace='rover_sim',
                    executable='offboard_sim_control',
                    name='velocity'
                )
            ]
        ),
    ])
