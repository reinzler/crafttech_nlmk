from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import subprocess

# List of commands to run
commands = [
    # Run the MicroROS - ROS2 Bridge
    "ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200",
]

# Loop through each command in the list

for command in commands:
    # Each command is run in a new tab of the gnome-terminal
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])

    # Pause between each command

def generate_launch_description():

    maths = Node(
        package='bumble_sim',
        executable='math',
        name='bumble_math',
        output='screen'
    )

    # Запуск ноды Телеоперации с клавиатуры в отдельном терминале
    keyboard_teleop_terminal = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'bumblebee_teleop', 'bumblebee_keyboard_teleop'],
        output='screen'
    )

    rviz2_diffbot_path = os.path.join(
        get_package_share_directory('ros2_control_demo_example_2'),
        'launch',
        'diffbot.launch.py'
    )

    # Запуск другого лаунч файла
    rviz2_diffbot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz2_diffbot_path),
        launch_arguments={'gui': LaunchConfiguration('gui'), 'use_mock_hardware': LaunchConfiguration('use_mock_hardware')}.items(),
    )

    # return LaunchDescription([maths, keyboard_teleop_terminal, rviz2_diffbot_node])
    return LaunchDescription([maths, keyboard_teleop_terminal])