#!/bin/bash

# 1. Identify where we are
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Move up to the workspace level (assuming src/internal/the_cruiser/scripts structure)
WS_ROOT="$(cd "$SCRIPT_DIR/../../../../" && pwd)"

echo "🤖 THE CRUISER: Automated Setup & Launch"
echo "📍 Workspace: $WS_ROOT"

# 2. Check for dependencies (TurtleBot3 & Nav2)
if [ ! -d "/opt/ros/humble/share/turtlebot3_navigation2" ]; then
    echo "❌ Error: TurtleBot3 Nav2 not found in /opt/ros/humble."
    echo "Please run: sudo apt update && sudo apt install ros-humble-turtlebot3-navigation2 ros-humble-turtlebot3-gazebo"
    exit 1
fi

# 3. Build the package if install folder is missing
if [ ! -d "$WS_ROOT/install" ]; then
    echo "🏗️  Building workspace for the first time..."
    cd "$WS_ROOT" && colcon build --packages-select the_cruiser
fi

# 4. Set environment variables
export TURTLEBOT3_MODEL=waffle
source /opt/ros/humble/setup.bash
source "$WS_ROOT/install/setup.bash"

# 5. Launch the tabs
echo "🚀 Launching Simulation, Nav2, and The Cruiser..."

# Tab 1: Gazebo
gnome-terminal --tab --title="1. Gazebo" -- bash -c "source /opt/ros/humble/setup.bash; export TURTLEBOT3_MODEL=waffle; ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py; exec bash"

sleep 4

# Tab 2: Nav2
gnome-terminal --tab --title="2. Nav2" -- bash -c "source /opt/ros/humble/setup.bash; export TURTLEBOT3_MODEL=waffle; ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/opt/ros/humble/share/turtlebot3_navigation2/map/map.yaml; exec bash"

sleep 4

# Tab 3: The Cruiser (Our Node)
gnome-terminal --tab --title="3. The Cruiser" -- bash -c "source $WS_ROOT/install/setup.bash; ros2 lifecycle set /controller_server deactivate; ros2 run the_cruiser cruiser_node; exec bash"

echo "✅ All systems active. Set a Nav Goal in Rviz!"
