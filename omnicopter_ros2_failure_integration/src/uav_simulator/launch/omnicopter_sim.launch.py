from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

 # Correct paths for models, worlds, and meshes in the source folder
    uav_simulator_src_path = os.path.join(os.getenv('ROS_WS', '/ros_ws'), 'src', 'uav_simulator')
    models_path = os.path.join(uav_simulator_src_path, 'models')
    worlds_path = os.path.join(uav_simulator_src_path, 'worlds')
    meshes_path = os.path.join(models_path, 'omnicopter', 'meshes')
    world_file = os.path.join(worlds_path, 'omnicopter.world')

    # Declare launch arguments
    mav_name = LaunchConfiguration('mav_name')
    world_name = LaunchConfiguration('world_name')
    gui = LaunchConfiguration('gui')
    paused = LaunchConfiguration('paused')
    init_z = LaunchConfiguration('init_z')
    verbose = LaunchConfiguration('verbose')

    # Launch Gazebo Sim
    gazebo = ExecuteProcess(
        cmd=[
            'gz', 'sim',
            world_file
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
        # Correct environment variable setting
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', f'{worlds_path}'),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', '/ros_ws/install/uav_simulator/lib'),
        gazebo
    ])
