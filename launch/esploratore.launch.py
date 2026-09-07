import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    explorer_pkg_dir = get_package_share_directory('explorer_aggr')

    # Path to custom Nav2 configuration parameters
    custom_params_file = os.path.join(explorer_pkg_dir, 'config', 'mio_nav2_params.yaml')

    # Launch Nav2 in a separate terminal process to isolate navigation logs
    nav2_terminal_cmd = [
        'gnome-terminal', '--', 'bash', '-c', 
        f'ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:={custom_params_file} ; exec bash'
    ]

    nav2_launch = ExecuteProcess(
        cmd=nav2_terminal_cmd,
        output='screen'
    )

    # Autonomous exploration communication and frontier selection node
    communication_node = Node(
        package='explorer_aggr',
        executable='communication_node',
        name='communication_node',
        output='screen',
        parameters=[{'use_sim_time': True}]  # Synchronize time with simulation clock
    )

    return LaunchDescription([
        nav2_launch,
        communication_node
    ])