from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # package_dir = FindPackageShare('bumble_sim').find('bumble_sim')

    maths = Node(
        package='bumble_sim',
        executable='math',
        name='bumble_math',
        output='screen'
    )

    keyboard_teleop = Node(
        package='bumblebee_teleop',
        executable='bumblebee_keyboard_teleop',
        name='bumble_keyboard_teleop',
        output='screen'
    )

    # Define the nodes to be launched
    return LaunchDescription([maths, keyboard_teleop])

