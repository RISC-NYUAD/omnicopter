def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'gz', 'sim',
                os.path.join(
                    os.getenv('GAZEBO_RESOURCE_PATH', os.getcwd()),
                    'worlds', 'empty.sdf'
                )
            ],
            output='screen'
        )
    ])