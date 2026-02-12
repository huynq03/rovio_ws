#!/bin/bash

# Stop all ROVIO-related processes

echo "════════════════════════════════════════════════════════════════"
echo "          Stopping ROVIO System"
echo "════════════════════════════════════════════════════════════════"
echo ""

# Kill ROVIO node
if pgrep -f "rovio_node" > /dev/null; then
    echo "🛑 Stopping ROVIO node..."
    pkill -f "rovio_node"
    sleep 1
fi

# Kill QoS relay
if pgrep -f "qos_relay.py" > /dev/null; then
    echo "🛑 Stopping QoS relay..."
    pkill -f "qos_relay.py"
    sleep 1
fi

# Kill RealSense camera
if pgrep -f "realsense2_camera_node" > /dev/null; then
    echo "🛑 Stopping RealSense camera..."
    pkill -f "realsense2_camera_node"
    pkill -f "rs_launch.py"
    sleep 1
fi

# Kill monitor script
if pgrep -f "monitor_realtime.sh" > /dev/null; then
    echo "🛑 Stopping monitor..."
    pkill -f "monitor_realtime.sh"
fi

echo ""
echo "✅ All ROVIO processes stopped"
echo ""

# Check if anything is still running
if pgrep -f "rovio|realsense|qos_relay" > /dev/null; then
    echo "⚠️  Warning: Some processes may still be running:"
    ps aux | grep -E "rovio|realsense|qos_relay" | grep -v grep
    echo ""
    echo "To force kill: pkill -9 -f 'rovio|realsense|qos_relay'"
else
    echo "✅ Clean shutdown completed"
fi
