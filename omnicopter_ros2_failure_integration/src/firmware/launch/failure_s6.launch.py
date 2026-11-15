"""Adopted two-stage failure controller; do not also launch simulate.launch.py."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    simulator = get_package_share_directory('uav_simulator')
    resource_path = os.path.join(simulator, 'worlds', 'models')
    plugin_path = os.path.join(get_package_prefix('uav_simulator'), 'lib')
    config = os.path.join(get_package_share_directory('controller'), 'config', 'failure_aware.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', '')),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_path + os.pathsep + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='s6_contact_bridge',
             arguments=['/omnicopter/s6/contact_base@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts', '/omnicopter/s6/contact_rotor_0@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts', '/omnicopter/s6/contact_rotor_1@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts', '/omnicopter/s6/contact_rotor_2@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts', '/omnicopter/s6/contact_rotor_3@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts', '/omnicopter/s6/contact_rotor_4@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts', '/omnicopter/s6/contact_rotor_5@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts', '/omnicopter/s6/contact_rotor_6@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts', '/omnicopter/s6/contact_rotor_7@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'], parameters=[{'use_sim_time': True}], output='screen'),
        # Server always starts unpaused; plugin is the single ROS /clock source.
        ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', os.path.join(simulator, 'worlds', 'failure_s6.world')], output='screen'),
        ExecuteProcess(cmd=['gz', 'sim', '-g'], condition=IfCondition(LaunchConfiguration('gui')), output='screen'),
        Node(package='controller', executable='failure_aware_node', name='failure_controller',
             parameters=[config, {'use_sim_time': True}], output='screen'),
        Node(package='maneuver', executable='maneuver_node', name='maneuver',
             parameters=[{'use_sim_time': True}], output='screen'),
    ])
