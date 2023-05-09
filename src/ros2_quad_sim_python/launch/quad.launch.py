import launch
import launch_ros.actions

#TODO: add all the parameters here...

def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='init_pose',
            default_value="[0,0,2,0,0,0]",
            description='Initial pose'
        ),
        launch.actions.DeclareLaunchArgument(
            name='target_frame',
            default_value='flying_sensor',
            description='Target frame for the flying sensor'
        ),
        launch.actions.DeclareLaunchArgument(
            name='map_frame',
            default_value='map',
            description='Target frame for the flying sensor'
        ),
        launch.actions.DeclareLaunchArgument(
            name='Px',
            default_value='5.0',
            description='Position P gain for x'
        ),
        launch.actions.DeclareLaunchArgument(
            name='Py',
            default_value='5.0',
            description='Position P gain for y'
        ),
        launch.actions.DeclareLaunchArgument(
            name='Pz',
            default_value='2.0',
            description='Position P gain for z'
        ),
        launch.actions.DeclareLaunchArgument(
            name='Pxdot',
            default_value='5.0',
            description='Velocity P gain for X'
        ),
        launch.actions.DeclareLaunchArgument(
            name='Pydot',
            default_value='5.0',
            description='Velocity P gain for Y'
        ),
        launch.actions.DeclareLaunchArgument(
            name='Pzdot',
            default_value='2.0',
            description='Velocity P gain for Z'
        ),
        launch.actions.DeclareLaunchArgument(
            name='tiltMax',
            default_value='30.0',
            description='Max tilt [degrees]'
        ),
        launch_ros.actions.Node(
            package='ros2_quad_sim_python',
            executable='quad',
            name='quadsim',
            output='screen',
            emulate_tty='True',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {
                    'init_pose': launch.substitutions.LaunchConfiguration('init_pose')
                },
                {
                    'target_frame': launch.substitutions.LaunchConfiguration('target_frame')
                },
                {
                    'map_frame': launch.substitutions.LaunchConfiguration('map_frame')
                },
                {
                    'Px': launch.substitutions.LaunchConfiguration('Px')
                },
                {
                    'Py': launch.substitutions.LaunchConfiguration('Py')
                },
                {
                    'Pz': launch.substitutions.LaunchConfiguration('Pz')
                },
                {
                    'Pxdot': launch.substitutions.LaunchConfiguration('Pxdot')
                },
                {
                    'Pydot': launch.substitutions.LaunchConfiguration('Pydot')
                },
                {
                    'Pzdot': launch.substitutions.LaunchConfiguration('Pzdot')
                },
                {
                    'tiltMax': launch.substitutions.LaunchConfiguration('tiltMax')
                },
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()