import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 1. Locate the packages
    pkg_turtlebot3 = get_package_share_directory("turtlebot3_gazebo")
    pkg_the_cruiser = get_package_share_directory("the_cruiser")

    # 2. Simulation (Standard TurtleBot3 World)
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3, "launch", "turtlebot3_world.launch.py")
        )
    )

    # 3. Your Controller Node (The Cruiser)
    # This is the node that fulfills your assignment requirements.
    cruiser_cmd = Node(
        package="the_cruiser",
        executable="cruiser_node",
        name="cruiser_node",
        output="screen",
        # You can set your controller parameters here
        parameters=[{"max_v": 0.7, "max_w": 1.5, "kp_lin": 1.2, "kp_ang": 2.0}],
    )

    # 4. Visualization (Rviz2)
    # We use a custom RViz configuration to easily publish a path.
    rviz_config_file = os.path.join(pkg_the_cruiser, "config", "cruiser.rviz")
    rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription([gazebo_cmd, cruiser_cmd, rviz_cmd])
