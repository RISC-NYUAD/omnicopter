from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'gz', 'sim',
                os.path.join(
                    os.getenv('GAZEBO_RESOURCE_PATH', '/usr/share/gz-sim8'),
                    'worlds', 'empty.sdf'
                )
            ],
            output='screen'
        )
    ])
