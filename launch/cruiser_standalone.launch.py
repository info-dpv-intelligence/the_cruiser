import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch The Cruiser Controller Node.

    NOTE: This launch file assumes Gazebo and Nav2 are already running!

    Run in this order:
    Terminal 1: ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    Terminal 2: ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/opt/ros/humble/share/turtlebot3_navigation2/map/map.yaml
    Terminal 3: ros2 launch the_cruiser cruiser_standalone.launch.py  <-- This file
    Terminal 4: (optional) ~/dev/ros2_ws/src/internal/the_cruiser/scripts/verify_control.sh
    """

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
    # LIFECYCLE COORDINATION
    # Configure Cruiser (UNCONFIGURED → CONFIGURED)
    # Deactivate Nav2 controller_server
    # Activate Cruiser (CONFIGURED → ACTIVE)
    # ========================================================================
    configure_cruiser = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/cruiser_node", "configure"],
                output="screen",
                shell=False,
            )
        ],
    )

    deactivate_nav2_controller = TimerAction(
        period=2.5,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/controller_server", "deactivate"],
                output="screen",
                shell=False,
            )
        ],
    )

    activate_cruiser = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/cruiser_node", "activate"],
                output="screen",
                shell=False,
            )
        ],
    )

    # ========================================================================
    # ASSEMBLE THE LAUNCH DESCRIPTION
    # ========================================================================
    return LaunchDescription(
        [
            cruiser_node,
            configure_cruiser,
            deactivate_nav2_controller,
            activate_cruiser,
        ]
    )
