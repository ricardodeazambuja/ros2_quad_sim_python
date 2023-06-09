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
            description='Map frame for the flying sensor'
        ),
        launch_ros.actions.Node(
            package='ros2_quad_sim_python',
            executable='quadsim',
            name='quadsim',
            output={'both': 'log'},
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
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()