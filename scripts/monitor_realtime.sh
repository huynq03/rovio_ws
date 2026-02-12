#!/bin/bash

# ROVIO Real-time Performance Monitor for Jetson Orin
# Usage: ./monitor_realtime.sh

echo "════════════════════════════════════════════════════════════════"
echo "         ROVIO Real-time Performance Monitor - Jetson Orin"
echo "════════════════════════════════════════════════════════════════"
echo ""

# Source ROS2
source ~/rovio_ws/install/setup.bash

# Check if ROVIO is running
if ! pgrep -f "rovio_node" > /dev/null; then
    echo "❌ ERROR: ROVIO node is not running!"
    echo "   Start with: ros2 run rovio rovio_node ..."
    exit 1
fi

echo "✅ ROVIO node detected"
echo ""

# Function to check topic
check_topic() {
    local topic=$1
    local name=$2
    if ros2 topic info "$topic" &>/dev/null; then
        echo "  ✅ $name"
    else
        echo "  ❌ $name NOT FOUND"
    fi
}

# Check topics
echo "📡 Checking Topics:"
check_topic "/cam0/image_raw" "Camera"
check_topic "/imu0" "IMU"
check_topic "/rovio/odometry" "Odometry"
echo ""

# Monitor in loop
echo "📊 Real-time Monitoring (Press Ctrl+C to stop):"
echo "────────────────────────────────────────────────────────────────"
echo ""

while true; do
    clear
    echo "════════════════════════════════════════════════════════════════"
    echo "         ROVIO Real-time Performance - $(date +%H:%M:%S)"
    echo "════════════════════════════════════════════════════════════════"
    echo ""
    
    # CPU Info
    echo "🖥️  CPU Status:"
    cpu_freq=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq)
    cpu_ghz=$(awk "BEGIN {printf \"%.2f\", $cpu_freq/1000000}")
    echo "   Frequency: ${cpu_ghz} GHz"
    
    # Process CPU usage
    echo ""
    echo "⚙️  Process Usage:"
    ps aux | grep -E "rovio_node|realsense|qos_relay" | grep -v grep | \
        awk '{printf "   %-20s CPU: %5s%%  MEM: %4s%%\n", substr($11,length($11)-19), $3, $4}'
    
    # Topic rates (quick check - 2 seconds)
    echo ""
    echo "📈 Topic Rates (2s sample):"
    
    # Camera rate
    cam_hz=$(timeout 2 ros2 topic hz /cam0/image_raw 2>/dev/null | grep "average rate:" | tail -1 | awk '{print $3}')
    if [ -n "$cam_hz" ]; then
        printf "   Camera:   %6.2f Hz\n" "$cam_hz"
    else
        echo "   Camera:     -- Hz"
    fi
    
    # Odometry rate
    odom_hz=$(timeout 2 ros2 topic hz /rovio/odometry 2>/dev/null | grep "average rate:" | tail -1 | awk '{print $3}')
    if [ -n "$odom_hz" ]; then
        printf "   Odometry: %6.2f Hz\n" "$odom_hz"
    else
        echo "   Odometry:   -- Hz"
    fi
    
    # Temperature
    echo ""
    echo "🌡️  Temperature:"
    if command -v tegrastats &> /dev/null; then
        temp=$(timeout 1 tegrastats --interval 100 2>/dev/null | head -1 | grep -oP 'CPU@\K[0-9.]+' | head -1)
        if [ -n "$temp" ]; then
            echo "   CPU: ${temp}°C"
        fi
    fi
    
    echo ""
    echo "────────────────────────────────────────────────────────────────"
    echo "Press Ctrl+C to stop monitoring"
    
    sleep 5
done
