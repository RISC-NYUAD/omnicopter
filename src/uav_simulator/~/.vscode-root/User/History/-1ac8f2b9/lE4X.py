from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    world_file = os.path.join(
        os.path.dirname(__file__), '..', 'worlds', 'empty.sdf')
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', world_file],
            output='screen'
        )
    ])