import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch The Cruiser Controller Node with RViz.

    NOTE: This launch file assumes Gazebo and Nav2 are already running!

    Run in this order:
    Terminal 1: ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    Terminal 2: ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/opt/ros/humble/share/turtlebot3_navigation2/map/map.yaml
    Terminal 3: ros2 launch the_cruiser cruiser_standalone.launch.py  <-- This file
    Terminal 4: (optional) ~/dev/ros2_ws/src/internal/the_cruiser/scripts/verify_control.sh
    """

    pkg_the_cruiser = get_package_share_directory("the_cruiser")

    # ========================================================================
    # THE CRUISER NODE (Lifecycle Node - starts in CONFIGURED state)
    # ========================================================================
    cruiser_node = Node(
        package="the_cruiser",
        executable="cruiser_node",
        name="cruiser_node",
        output="screen",
        parameters=[
            {"max_v": 0.7},
            {"max_w": 1.5},
            {"kp_lin": 1.2},
            {"kp_ang": 2.0},
        ],
    )

    # ========================================================================
    # RVIZ VISUALIZATION
    # ========================================================================
    rviz_config_file = os.path.join(pkg_the_cruiser, "config", "cruiser.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    # ========================================================================
    # LIFECYCLE COORDINATION
    # Deactivate Nav2 controller_server, activate Cruiser
    # ========================================================================
    deactivate_nav2_controller = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/controller_server", "deactivate"],
        output="screen",
        shell=False,
    )

    activate_cruiser = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/cruiser_node", "activate"],
        output="screen",
        shell=False,
    )

    # ========================================================================
    # ASSEMBLE THE LAUNCH DESCRIPTION
    # ========================================================================
    return LaunchDescription(
        [
            cruiser_node,
            rviz_node,
            deactivate_nav2_controller,
            activate_cruiser,
        ]
    )
