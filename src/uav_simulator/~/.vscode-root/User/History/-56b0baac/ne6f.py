from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Declare launch arguments
    mav_name = LaunchConfiguration('mav_name', default='omnicopter')
    world_name = LaunchConfiguration('world_name', default='omnicopter')
    gui = LaunchConfiguration('gui', default='true')
    paused = LaunchConfiguration('paused', default='true')
    verbose = LaunchConfiguration('verbose', default='true')

    # Resolve paths
    world_path = os.path.join(
        get_package_share_directory('uav_simulator'), 'worlds', f'{world_name.perform(None)}.world')

    # Include Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'paused': paused,
            'gui': gui,
            'verbose': verbose
        }.items()
    )

    # Adapter node
    adapter_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('uav_simulator'), 'launch', 'adapter_node.launch.py')
        ),
        launch_arguments={
            'mav_name': mav_name
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('mav_name', default_value='omnicopter'),
        DeclareLaunchArgument('world_name', default_value='omnicopter'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='true'),
        gazebo,
        adapter_node
    ])