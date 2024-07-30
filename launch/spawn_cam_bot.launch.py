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
    urdf_visualise = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(publish_robot_package, 'launch',
                         'cam_bot_description.launch.py'),
        )
    )

    # Will spawn in Gazebo whatever there is in the /robot_description topic
    spawn_robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(publish_robot_package, 'launch',
                         'cam_bot_spawn.launch.py'),
        )
    )

    

    tf_cam_pub = Node(
            package='turtle_tf_3d_ros2',
            executable='cam_bot_odom_to_tf_pub.py',
            output='screen',
            name='cam_bot_odom_to_tf_pub_node',
            emulate_tty=True,
            parameters=[{'use_sim_time': True}])

    
    cmd_vel_cam_bot = Node(
            package='turtle_tf_3d_ros2',
            executable='cam_bot_force.py',
            output='screen',
            name='cam_bot_force_node',
            emulate_tty=True,
            parameters=[{'use_sim_time': True}])



    # create and return launch description object
    return LaunchDescription(
        [
            urdf_visualise,
            spawn_robot_description,
            tf_cam_pub,
            cmd_vel_cam_bot
        ]
    )