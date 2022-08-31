import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# You need to install (or have in your workspace):
# https://github.com/ricardodeazambuja/depthimage_to_pointcloud2

def generate_launch_description():
    sensor_prefix = ['back', 'front', 'right', 'left', 'down']
    descriptions = []
    for sensor in sensor_prefix:
        description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('depthimage_to_pointcloud2'), 'launch/depthimage_to_pointcloud2.launch.py')]),
                launch_arguments={'full_sensor_topic': f'/carla/flying_sensor/depth_{sensor}', 
                                  'rgb_image_topic': f'/carla/flying_sensor/semantic_segmentation_{sensor}',
                                  'use_quiet_nan': 'true',
                                  'range_max': '19.0'}.items()
            )
        descriptions.append(description)
    return LaunchDescription(descriptions)


if __name__ == "__main__":
    generate_launch_description()