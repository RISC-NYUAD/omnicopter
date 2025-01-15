from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    world_file = "/usr/share/gz/gz-sim8/worlds/empty.sdf"
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-s', world_file],  # '-s' disables GUI
            output='screen'
        )
    ])