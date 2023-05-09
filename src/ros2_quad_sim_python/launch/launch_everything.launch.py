import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
   town = LaunchConfiguration('town')
   town_launch_arg = DeclareLaunchArgument(
      'town',
      default_value='Town01'
   )
   carla_bridge = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('carla_ros_bridge'), 'carla_ros_bridge.launch.py')]
      ),
      launch_arguments={'town': town}.items(),
      )
   objects_definition_file = LaunchConfiguration('objects_definition_file')
   objects_definition_file_arg = DeclareLaunchArgument(
      'objects_definition_file',
      default_value=os.path.join(get_package_share_directory('ros2_quad_sim_python'),'cfg/flying_sensor.json')
   )
   carla_spawn_objects = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('carla_spawn_objects'), 'carla_spawn_objects.launch.py')]),
      launch_arguments={'objects_definition_file': objects_definition_file}.items(),
      )
   
   quad_params = {
         'target_frame': 'flying_sensor', 
         'map_frame': 'map', 
         'init_pose': '[0,0,2,0,0,0]',
         'Px': '5.0',
         'Py': '5.0',
         'Pz': '2.0',
         'Pxdot': '5.0',
         'Pydot': '5.0',
         'Pzdot': '2.0',
         'tiltMax': '30.0',
         }
   quad_params_lcgf = {k: LaunchConfiguration(k) for k in quad_params.keys()}
   quad_params_arg = [DeclareLaunchArgument(k, default_value=v) for k,v in quad_params.items()]
   quad = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('ros2_quad_sim_python'), 'quad.launch.py')]),
      launch_arguments=quad_params_lcgf.items(),
      )

   return LaunchDescription([
      town_launch_arg,
      carla_bridge,
      objects_definition_file_arg,
      carla_spawn_objects,
      *quad_params_arg,
      quad,
      Node(
         package='rviz2',
         executable='rviz2',
         name='rviz2',
         arguments=['-d', os.path.join(get_package_share_directory('ros2_quad_sim_python'), 'cfg/rviz_flying_sensor.rviz')]
      )
   ])