import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription, ExecuteProcess, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_cruiser(context: LaunchContext, *args, **kwargs):
    """
    Deactivate Nav2's controller_server and activate Cruiser
    This is called after all nodes have been spawned to ensure proper coordination.
    """
    actions = [
        ExecuteProcess(
            cmd=["ros2", "lifecycle", "set", "/controller_server", "deactivate"],
            output="screen",
            shell=False,
        ),
        ExecuteProcess(
            cmd=["ros2", "lifecycle", "set", "/cruiser_node", "activate"],
            output="screen",
            shell=False,
        ),
    ]
    return actions


def generate_launch_description():
    """
    Complete launch description for The Cruiser with Nav2.

    Workflow:
    1. Start Gazebo simulation (TurtleBot3)
    2. Start Nav2 (with controller_server in CONFIGURED state)
    3. Start Cruiser node (in CONFIGURED state)
    4. Deactivate Nav2 controller_server
    5. Activate Cruiser node to take control
    """

    # Get package directories
    pkg_turtlebot3_gazebo = get_package_share_directory("turtlebot3_gazebo")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")
    pkg_the_cruiser = get_package_share_directory("the_cruiser")

    # ========================================================================
    # 1. GAZEBO SIMULATION
    # ========================================================================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, "launch", "turtlebot3_world.launch.py")
        )
    )

    # ========================================================================
    # 2. NAV2 BRINGUP (with lifecycle management)
    # ========================================================================
    nav2_params = os.path.join(pkg_nav2_bringup, "params", "nav2_params.yaml")
    nav2_map = "/opt/ros/humble/share/turtlebot3_navigation2/map/map.yaml"

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, "launch", "navigation_launch.py")
        ),
        launch_arguments=[
            ("use_sim_time", "True"),
            ("params_file", nav2_params),
            ("map", nav2_map),
        ],
    )

    # ========================================================================
    # 3. THE CRUISER NODE (Lifecycle Node - starts in CONFIGURED state)
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
    # 4. RVIZ VISUALIZATION
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
    # 5. LIFECYCLE COORDINATION (deactivate Nav2, activate Cruiser)
    # This runs after all nodes are spawned
    # ========================================================================
    lifecycle_coordination = OpaqueFunction(function=launch_cruiser)

    # ========================================================================
    # ASSEMBLE THE LAUNCH DESCRIPTION
    # ========================================================================
    return LaunchDescription(
        [
            gazebo_launch,
            nav2_launch,
            cruiser_node,
            rviz_node,
            lifecycle_coordination,
        ]
    )
