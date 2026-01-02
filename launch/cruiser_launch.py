import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Locate the packages
    pkg_turtlebot3 = get_package_share_directory('turtlebot3_gazebo')

    # 2. Simulation (Standard TurtleBot3 World)
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # 3. YOUR Controller Node (The Cruiser)
    cruiser_cmd = Node(
        package='the_cruiser',
        executable='cruiser_node',
        name='cruiser_node',
        output='screen',
        parameters=[{
            'max_v': 1.2,
            'max_w': 3.0,
            'kp_lin': 2.0,
            'kp_ang': 6.0
        }]
    )

    # 4. Visualization (Rviz2)
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        gazebo_cmd,
        cruiser_cmd,
        rviz_cmd
    ])
