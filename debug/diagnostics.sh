cat > ~/dev/ros2_ws/diagnostics.sh << 'EOF'
#!/bin/bash

echo "=========================================="
echo "CRUISER SYSTEM DIAGNOSTICS"
echo "=========================================="
echo ""

# Terminal 1: Gazebo + Nav2 Planner
echo "📋 [TERMINAL 1] Starting Gazebo & Nav2..."
echo "   Command: ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True params_file:=/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml map:=/opt/ros/humble/share/turtlebot3_navigation2/map/map.yaml"
echo ""
echo "   ⏳ Wait ~30 seconds for Gazebo to fully load"
echo ""

EOF

chmod +x ~/dev/ros2_ws/diagnostics.sh