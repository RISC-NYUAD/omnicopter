from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    world_file = "/ros_ws/src/uav_simulator/worlds/empty.sdf"
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-s', world_file],  # '-s' disables GUI
            output='screen'
        )
    ])