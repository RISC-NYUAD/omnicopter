from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Use absolute path inside the container
    current_dir = os.path.dirname(os.path.realpath(__file__))
    world_file = os.path.join(current_dir, '..', 'worlds', 'empty.sdf')
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', world_file],
            output='screen'
        )
    ])
