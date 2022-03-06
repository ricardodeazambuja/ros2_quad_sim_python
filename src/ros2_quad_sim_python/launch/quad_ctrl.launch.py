import launch
import launch_ros.actions

#TODO: add all the parameters here...
def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='Px',
            default_value='2.0',
            description='Position P gain for x'
        ),
        launch.actions.DeclareLaunchArgument(
            name='Py',
            default_value='2.0',
            description='Position P gain for y'
        ),
        launch.actions.DeclareLaunchArgument(
            name='Pz',
            default_value='1.0',
            description='Position P gain for z'
        ),
        launch_ros.actions.Node(
            package='ros2_quad_sim_python',
            executable='quadctrl',
            name='quadctrl',
            output='screen',
            emulate_tty='True',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {
                    'Px': launch.substitutions.LaunchConfiguration('Px')
                },
                {
                    'Py': launch.substitutions.LaunchConfiguration('Py')
                },
                {
                    'Pz': launch.substitutions.LaunchConfiguration('Pz')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()