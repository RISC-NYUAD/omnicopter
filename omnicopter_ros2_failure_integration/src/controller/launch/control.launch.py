from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments with default values
    pose_arg = DeclareLaunchArgument('pose_full', default_value='pose_full', description='Pose topic name')
    pose_d_arg = DeclareLaunchArgument('pose_d', default_value='pose_d', description='Desired pose topic name')
    prop_cmd_arg = DeclareLaunchArgument('prop_cmd', default_value='prop_cmd', description='Propeller command topic name')
    prop_act_arg = DeclareLaunchArgument('controller_log', default_value='controller_log', description='Propeller actuator topic name')
    running_mode = DeclareLaunchArgument('mode', default_value='sim', description='Simulation or real experiment')

    # directory containing the YAML file
    config_directory = os.path.join('/ros_ws/src/controller/config', 'controller_gains.yaml')

    # Define the controller node
    controller_node = Node(
        package='controller',
        executable='controller_node',
        name='controller',
        output='screen',
        parameters=[
            config_directory,
            {'controller/pose_full': LaunchConfiguration('pose_full')},
            {'controller/pose_d': LaunchConfiguration('pose_d')},
            {'controller/prop_cmd': LaunchConfiguration('prop_cmd')},
            {'controller/log_data': LaunchConfiguration('controller_log')},
            {'controller/mode': LaunchConfiguration('mode')}      
        ]
    )

    return LaunchDescription([
        pose_arg,
        pose_d_arg,
        prop_cmd_arg,
        prop_act_arg,
        running_mode,
        controller_node
    ])
