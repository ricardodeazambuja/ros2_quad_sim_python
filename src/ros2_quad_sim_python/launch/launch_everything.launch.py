import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
   carla_bridge = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('carla_ros_bridge'), 'carla_ros_bridge.launch.py')]
      ))
   carla_spawn_objects = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('carla_spawn_objects'), 'carla_spawn_objects.launch.py')]),
      launch_arguments={'objects_definition_file': os.path.join(
         get_package_share_directory('ros2_quad_sim_python'), 'cfg/flying_sensor.json')}.items(),
      )
   quad = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('ros2_quad_sim_python'), 'quad.launch.py')]),
      launch_arguments={
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
         }.items(),
      )

   return LaunchDescription([
      carla_bridge,
      carla_spawn_objects,
      quad,
      Node(
         package='rviz2',
         executable='rviz2',
         name='rviz2',
         arguments=['-d', os.path.join(get_package_share_directory('ros2_quad_sim_python'), 'cfg/rviz_flying_sensor.rviz')]
      )
   ])