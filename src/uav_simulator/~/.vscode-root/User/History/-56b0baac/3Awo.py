from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Declare launch arguments
    mav_name = LaunchConfiguration('mav_name')
    world_name = LaunchConfiguration('world_name')
    gui = LaunchConfiguration('gui')
    paused = LaunchConfiguration('paused')
    init_z = LaunchConfiguration('init_z')
    verbose = LaunchConfiguration('verbose')

    # Path resolutions using PathJoinSubstitution
    uav_simulator_path = get_package_share_directory('uav_simulator')
    world_file = PathJoinSubstitution([uav_simulator_path, 'worlds', f'{world_name}.world'])
    params_file = PathJoinSubstitution([uav_simulator_path, 'config', 'omnicopter_params.yaml'])
    models_path = PathJoinSubstitution([uav_simulator_path, 'models'])

    # Launch Gazebo Sim
    gazebo = ExecuteProcess(
        cmd=[
            'gz', 'sim',
            '--world', world_file,
            '--verbose', verbose,
            '--gui', gui,
            '--pause' if paused == 'true' else ''
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('mav_name', default_value='omnicopter'),
        DeclareLaunchArgument('world_name', default_value='omnicopter'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='true'),
        DeclareLaunchArgument('init_z', default_value='0.2'),
        DeclareLaunchArgument('verbose', default_value='true'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', models_path),
        gazebo
    ])
