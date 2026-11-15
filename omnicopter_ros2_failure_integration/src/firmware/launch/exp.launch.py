from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory  # Import this module
import os

def generate_launch_description():
    # Declare mode argument for control.launch.py
    mode_arg = DeclareLaunchArgument(
        'mode', 
        default_value='exp', 
        description='Mode for the control system (e.g., sim or exp)'
    )

    # Paths to the other launch files
    uav_simulator_launch_file = os.path.join(
        get_package_share_directory('uav_simulator'), 
        'launch', 
        'omnicopter_sim.launch.py'
    )

    maneuver_launch_file = os.path.join(
        get_package_share_directory('maneuver'), 
        'launch', 
        'maneuver.launch.py'
    )

    control_launch_file = os.path.join(
        get_package_share_directory('controller'), 
        'launch', 
        'control.launch.py'
    )

    # Include other launch files
    uav_simulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(uav_simulator_launch_file)
    )

    maneuver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(maneuver_launch_file)
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(control_launch_file),
        launch_arguments={'mode': LaunchConfiguration('mode')}.items()
    )

    return LaunchDescription([
        # Declare arguments
        mode_arg,

        # Include launch files
        uav_simulator_launch,
        maneuver_launch,
        control_launch,
    ])
