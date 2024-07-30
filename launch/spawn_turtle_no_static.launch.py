import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# this is the function launch  system will look for


def generate_launch_description():


    publish_robot_package = "spawn_robot_tools_pkg"
    publish_robot_package = get_package_share_directory(publish_robot_package)
    
    # It will publish the URDF in /robot_description topic and open rviz
    urdf_turtle_visualise = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(publish_robot_package, 'launch',
                         'turtle_description.launch.py'),
        )
    )

    # Will spawn in Gazebo whatever there is in the /robot_description topic
    spawn_turtle_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(publish_robot_package, 'launch',
                         'turtle_spawn.launch.py'),
        )
    )

    # create and return launch description object
    return LaunchDescription(
        [
            urdf_turtle_visualise,
            spawn_turtle_description,
        ]
    )