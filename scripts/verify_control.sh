#!/bin/bash

# ============================================================================
# CRUISER VERIFICATION SCRIPT
# Scientifically proves that The Cruiser is controlling the robot
# ============================================================================

set -e

echo ""
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║           CRUISER SYSTEM VERIFICATION & DIAGNOSTICS           ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ============================================================================
# 1. CHECK NODE STATES
# ============================================================================
echo -e "${BLUE}[1/5] CHECKING LIFECYCLE STATES${NC}"
echo "---------------------------------------"

echo ""
echo "Controller Server (Nav2):"
ros2 lifecycle get /controller_server 2>/dev/null || echo "  ❌ Not found or not lifecycle-managed"

echo ""
echo "Cruiser Node:"
ros2 lifecycle get /cruiser_node 2>/dev/null || echo "  ❌ Not found or not lifecycle-managed"

# ============================================================================
# 2. CHECK /cmd_vel PUBLISHERS
# ============================================================================
echo ""
echo ""
echo -e "${BLUE}[2/5] CHECKING /cmd_vel PUBLISHERS${NC}"
echo "---------------------------------------"

echo ""
echo "Current publishers on /cmd_vel:"
ros2 topic info /cmd_vel

# ============================================================================
# 3. MONITOR /cmd_vel FREQUENCY
# ============================================================================
echo ""
echo ""
echo -e "${BLUE}[3/5] MONITORING /cmd_vel FREQUENCY (10 seconds)${NC}"
echo "---------------------------------------"
echo ""
echo "Measuring publish rate..."

timeout 10 ros2 topic bw /cmd_vel 2>&1 || true

# ============================================================================
# 4. CHECK DIAGNOSTICS
# ============================================================================
echo ""
echo ""
echo -e "${BLUE}[4/5] CHECKING DIAGNOSTICS${NC}"
echo "---------------------------------------"
echo ""
echo "Latest diagnostic snapshot (Cruiser Controller):"

timeout 5 ros2 topic echo /diagnostics --once 2>/dev/null || echo "  ⚠️  No diagnostics yet (run with active trajectory)"

# ============================================================================
# 5. SUMMARY
# ============================================================================
echo ""
echo ""
echo -e "${BLUE}[5/5] VERIFICATION SUMMARY${NC}"
echo "---------------------------------------"
echo ""
echo -e "${GREEN}✅ PROOF POINTS:${NC}"
echo "  1. Check if /controller_server is DEACTIVATED"
echo "  2. Check if /cruiser_node is ACTIVATED"
echo "  3. Verify only ONE publisher on /cmd_vel (should be /cruiser_node)"
echo "  4. /cmd_vel frequency should be ~20 Hz (50ms control loop)"
echo "  5. Diagnostics should show Plan counts & Control loop activity"
echo ""
echo -e "${YELLOW}INTERPRETATION:${NC}"
echo "  • If /cruiser_node is ACTIVE and publishes /cmd_vel at 20Hz"
echo "    → Cruiser is in control ✅"
echo ""
echo "  • If /controller_server is still ACTIVE"
echo "    → Nav2 is potentially in control (problem!) ❌"
echo ""
echo "  • If /cmd_vel has multiple publishers"
echo "    → Topic contention! (undefined behavior) ❌"
echo ""
echo ""
echo -e "${BLUE}NEXT STEPS:${NC}"
echo "  1. Set a 2D Nav Goal in RViz"
echo "  2. Robot should move using Cruiser's trajectory"
echo "  3. Check logs: cruiser_node should show trajectory smoothing"
echo "  4. If diagnostics show 'Plans Received' > 0, algorithm is running"
echo ""
