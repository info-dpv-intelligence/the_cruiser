#!/bin/bash

# THE CRUISER DEMO LAUNCHER
echo "🚀 Initializing The Cruiser Demo Stack..."

# 1. Launch Simulation
gnome-terminal --tab --title="Gazebo" -- bash -c "source /opt/ros/humble/setup.bash; export TURTLEBOT3_MODEL=waffle; ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py; exec bash"

sleep 5

# 2. Launch Nav2
gnome-terminal --tab --title="Nav2" -- bash -c "source /opt/ros/humble/setup.bash; export TURTLEBOT3_MODEL=waffle; ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/opt/ros/humble/share/turtlebot3_navigation2/map/map.yaml; exec bash"

sleep 5

# 3. The Swap & Launch Cruiser
gnome-terminal --tab --title="The Cruiser" -- bash -c "source ~/dev/ros2_ws/install/setup.bash; echo 'Killing Standard Controller...'; ros2 lifecycle set /controller_server deactivate; echo 'Starting Cruiser Node...'; ros2 run the_cruiser cruiser_node; exec bash"

echo "✅ System Ready!"
