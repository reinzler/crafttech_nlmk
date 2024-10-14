from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Declare the arguments for the launch
    declared_arguments = [
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    ]

    # Include the diffbot launch file
    diffbot_launch_path = os.path.join(
        get_package_share_directory('ros2_control_demo_example_2'),
        'launch',
        'diffbot.launch.py'
    )

    diffbot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(diffbot_launch_path),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
        }.items(),
    )

    # Node maths from bumble_sim package
    maths = Node(
        package='bumble_sim',
        executable='math',
        name='bumble_math',
        output='screen'
    )

    # Open a new terminal tab for keyboard_teleop node
    keyboard_teleop_terminal = ExecuteProcess(
        cmd=['gnome-terminal', '--tab', '--', 'ros2', 'run', 'bumblebee_teleop', 'bumblebee_keyboard_teleop'],
        output='screen'
    )

    # Return the launch description with the diffbot launch, maths node, and teleop terminal
    return LaunchDescription(declared_arguments + [diffbot_launch, maths, keyboard_teleop_terminal])




