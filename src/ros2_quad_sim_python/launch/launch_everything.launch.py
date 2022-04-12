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
   quad_sim = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('ros2_quad_sim_python'), 'quad_sim.launch.py')]),
      launch_arguments={'target_frame': 'flying_sensor', 'map_frame': 'map', 'init_pose': '[0,0,2,0,0,0]'}.items(),
      )
   quad_ctrl = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('ros2_quad_sim_python'), 'quad_ctrl.launch.py')]),
      launch_arguments={'Px': '2.0', 'Py': '2.0', 'Pz': '1.0'}.items(),
      )

   return LaunchDescription([
      carla_bridge,
      carla_spawn_objects,
      quad_sim,
      quad_ctrl,
      Node(
         package='rviz2',
         executable='rviz2',
         name='rviz2',
         arguments=['-d', os.path.join(get_package_share_directory('ros2_quad_sim_python'), 'cfg/rviz_flying_sensor.rviz')]
      )
   ])