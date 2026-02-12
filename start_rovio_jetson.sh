#!/bin/bash

# ════════════════════════════════════════════════════════════════
#   ROVIO Auto-Start Script for Jetson Orin
#   Launches all required components in 4 separate terminals
# ════════════════════════════════════════════════════════════════

set -e

cd ~/rovio_ws

echo "════════════════════════════════════════════════════════════════"
echo "          Starting ROVIO System on Jetson Orin"
echo "════════════════════════════════════════════════════════════════"
echo ""

# Check if workspace is built
if [ ! -d "install" ]; then
    echo "❌ ERROR: Workspace not built!"
    echo "Please run: colcon build --symlink-install"
    exit 1
fi

# Check if RealSense camera is connected
if ! lsusb | grep -q "Intel"; then
    echo "⚠️  WARNING: RealSense camera not detected!"
    echo "Please plug in the camera and try again."
    read -p "Continue anyway? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Check if we need to enable max performance
read -p "Enable MAXN performance mode? (y/N): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "🚀 Enabling MAXN performance mode..."
    sudo ~/rovio_ws/scripts/jetson_performance.sh max
    echo ""
fi

echo "✅ Launching ROVIO in 4 terminals..."
echo ""
echo "Terminal 1: RealSense Camera"
echo "Terminal 2: QoS Relay (CRITICAL for topic bridging)"  
echo "Terminal 3: ROVIO Node"
echo "Terminal 4: Odometry Display"
echo ""
echo "Press Ctrl+C in any terminal to stop all components"
echo ""

sleep 2

# Check if gnome-terminal is available
if command -v gnome-terminal &> /dev/null; then
    # Launch 4 separate terminal windows
    echo "🚀 Launching Terminal 1: RealSense Camera..."
    gnome-terminal --title="ROVIO: Camera" --geometry=80x30+0+0 -- bash -c 'echo "═══════════════════════════════════════════════════════════════"; echo "  TERMINAL 1: RealSense D435i Camera"; echo "═══════════════════════════════════════════════════════════════"; echo ""; echo "Starting camera (minimal topics)..."; echo ""; cd ~/rovio_ws; source install/setup.bash; ros2 launch realsense2_camera rs_launch.py enable_color:=false enable_depth:=false enable_infra1:=true enable_infra2:=false enable_gyro:=true enable_accel:=true unite_imu_method:=1 infra_width:=640 infra_height:=480 infra_fps:=30 gyro_fps:=200 accel_fps:=200 initial_reset:=true publish_tf:=false enable_sync:=false; exec bash' &
    
    sleep 1
    echo "🚀 Launching Terminal 2: QoS Relay..."
    gnome-terminal --title="ROVIO: QoS Relay" --geometry=80x30+850+0 -- bash -c 'echo "═══════════════════════════════════════════════════════════════"; echo "  TERMINAL 2: QoS Relay (BEST_EFFORT → RELIABLE)"; echo "═══════════════════════════════════════════════════════════════"; echo ""; echo "Waiting for camera to initialize..."; sleep 5; echo "Starting QoS relay on CPU cores 4-7..."; echo ""; echo "Bridging topics:"; echo "  /camera/camera/infra1/image_rect_raw → /cam0/image_raw"; echo "  /camera/camera/imu → /imu0"; echo ""; cd ~/rovio_ws; source install/setup.bash; taskset -c 4-7 python3 src/rovio/scripts/qos_relay.py; exec bash' &
    
    sleep 1
    echo "🚀 Launching Terminal 3: ROVIO Node..."
    gnome-terminal --title="ROVIO: Main Node" --geometry=80x30+0+450 -- bash -c 'echo "═══════════════════════════════════════════════════════════════"; echo "  TERMINAL 3: ROVIO Node (Performance Cores)"; echo "═══════════════════════════════════════════════════════════════"; echo ""; echo "Waiting for camera and QoS relay..."; sleep 7; echo "Starting ROVIO on CPU cores 4-11..."; echo ""; echo "Subscribing to:"; echo "  /imu0 (from QoS relay)"; echo "  /cam0/image_raw (from QoS relay)"; echo ""; echo "⚠️  IMPORTANT: Move camera slowly with rotation to initialize!"; echo ""; cd ~/rovio_ws; source install/setup.bash; taskset -c 4-11 ros2 run rovio rovio_node --ros-args -p filter_config:=$(ros2 pkg prefix rovio)/share/rovio/cfg/rovio.info -p imu_topic:=/imu0 -p cam0_topic:=/cam0/image_raw; exec bash' &
    
    sleep 1
    echo "🚀 Launching Terminal 4: Odometry Display..."
    gnome-terminal --title="ROVIO: Odometry" --geometry=80x30+850+450 -- bash -c 'echo "═══════════════════════════════════════════════════════════════"; echo "  TERMINAL 4: Odometry Display"; echo "═══════════════════════════════════════════════════════════════"; echo ""; echo "Waiting for ROVIO to start publishing..."; cd ~/rovio_ws; source install/setup.bash; sleep 10; echo ""; echo "📊 Odometry Position (live update):"; echo "───────────────────────────────────────────────────────────════"; echo ""; ros2 topic echo /rovio/odometry --field pose.pose.position; exec bash' &
    
    sleep 2
    echo ""
    echo "✅ All 4 terminals launched!"
    echo ""
    echo "════════════════════════════════════════════════════════════════"
    echo "  📌 System Status:"
    echo "════════════════════════════════════════════════════════════════"
    echo "  Window 1 (Top-Left)    : Camera streaming @ 30 Hz + IMU @ 200 Hz"
    echo "  Window 2 (Top-Right)   : QoS relay bridging topics (cores 4-7)"
    echo "  Window 3 (Bottom-Left) : ROVIO processing & tracking (cores 4-11)"
    echo "  Window 4 (Bottom-Right): Live odometry position output"
    echo ""
    echo "════════════════════════════════════════════════════════════════"
    echo "  💡 Tips for Best Performance:"
    echo "════════════════════════════════════════════════════════════════"
    echo "  1. Wait 10-15 seconds for full initialization"
    echo "  2. Slowly move camera forward/backward with rotation"
    echo "  3. Point at textured surfaces (not blank walls)"
    echo "  4. Watch Terminal 3 for tracking initialization"
    echo ""
    echo "════════════════════════════════════════════════════════════════"
    echo "  🔧 Useful Commands (run in new terminal):"
    echo "════════════════════════════════════════════════════════════════"
    echo "  # Check odometry rate:"
    echo "    ros2 topic hz /rovio/odometry"
    echo ""
    echo "  # Monitor performance:"
    echo "    ~/rovio_ws/scripts/monitor_realtime.sh"
    echo ""
    echo "  # Check all topics:"
    echo "    ros2 topic list | grep rovio"
    echo ""
    echo "  # Stop all processes:"
    echo "    ~/rovio_ws/stop_rovio_jetson.sh"
    echo ""
    echo "════════════════════════════════════════════════════════════════"
    echo "  Expected Results: 30-40 Hz odometry, <20ms latency"
    echo "════════════════════════════════════════════════════════════════"
    echo ""
elif command -v tmux &> /dev/null; then
    # Use tmux as fallback
    SESSION="rovio"
    
    # Kill existing session if it exists
    tmux has-session -t $SESSION 2>/dev/null && tmux kill-session -t $SESSION
    
    # Create new session with first window
    tmux new-session -d -s $SESSION -n "Camera"
    
    # Window 0: RealSense Camera
    tmux send-keys -t $SESSION:0 "cd ~/rovio_ws && source install/setup.bash" C-m
    tmux send-keys -t $SESSION:0 "echo '═══ RealSense Camera ═══'" C-m
    tmux send-keys -t $SESSION:0 "ros2 launch realsense2_camera rs_launch.py enable_color:=false enable_depth:=false enable_infra1:=true enable_infra2:=false enable_gyro:=true enable_accel:=true unite_imu_method:=1 infra_width:=640 infra_height:=480 infra_fps:=30 gyro_fps:=200 accel_fps:=200 initial_reset:=true publish_tf:=false enable_sync:=false" C-m
    
    # Window 1: QoS Relay
    tmux new-window -t $SESSION:1 -n "QoS Relay"
    tmux send-keys -t $SESSION:1 "cd ~/rovio_ws && source install/setup.bash" C-m
    tmux send-keys -t $SESSION:1 "echo '═══ QoS Relay ═══'; sleep 5" C-m
    tmux send-keys -t $SESSION:1 "taskset -c 4-7 python3 src/rovio/scripts/qos_relay.py" C-m
    
    # Window 2: ROVIO Node
    tmux new-window -t $SESSION:2 -n "ROVIO"
    tmux send-keys -t $SESSION:2 "cd ~/rovio_ws && source install/setup.bash" C-m
    tmux send-keys -t $SESSION:2 "echo '═══ ROVIO Node ═══'; echo '⚠️  Move camera to initialize!'; sleep 7" C-m
    tmux send-keys -t $SESSION:2 "taskset -c 4-11 ros2 run rovio rovio_node --ros-args -p filter_config:=\$(ros2 pkg prefix rovio)/share/rovio/cfg/rovio.info -p imu_topic:=/imu0 -p cam0_topic:=/cam0/image_raw" C-m
    
    # Window 3: Odometry Display
    tmux new-window -t $SESSION:3 -n "Odometry"
    tmux send-keys -t $SESSION:3 "cd ~/rovio_ws && source install/setup.bash" C-m
    tmux send-keys -t $SESSION:3 "echo '═══ Odometry Output ═══'; sleep 10" C-m
    tmux send-keys -t $SESSION:3 "ros2 topic echo /rovio/odometry --field pose.pose.position" C-m
    
    # Select first window
    tmux select-window -t $SESSION:0
    
    echo ""
    echo "✅ TMUX session '$SESSION' created with 4 windows!"
    echo ""
    echo "════════════════════════════════════════════════════════════════"
    echo "  📌 TMUX Controls:"
    echo "════════════════════════════════════════════════════════════════"
    echo "  Switch windows : Ctrl+B then 0/1/2/3"
    echo "  Detach session : Ctrl+B then D"
    echo "  Reattach       : tmux attach -t $SESSION"
    echo "  Kill session   : tmux kill-session -t $SESSION"
    echo ""
    echo "════════════════════════════════════════════════════════════════"
    echo "  📊 Windows:"
    echo "════════════════════════════════════════════════════════════════"
    echo "  0: RealSense Camera (30 Hz + IMU 200 Hz)"
    echo "  1: QoS Relay (cores 4-7) - Bridges BEST_EFFORT → RELIABLE"
    echo "  2: ROVIO Node (cores 4-11) - ⚠️  Move camera to initialize!"
    echo "  3: Odometry Display"
    echo ""
    echo "════════════════════════════════════════════════════════════════"
    echo "  Expected Results: 30-40 Hz odometry, <20ms latency"
    echo "════════════════════════════════════════════════════════════════"
    echo ""
    
    # Attach to session
    tmux attach -t $SESSION

else
    echo "════════════════════════════════════════════════════════════════"
    echo "  ❌ ERROR: No supported terminal emulator found!"
    echo "════════════════════════════════════════════════════════════════"
    echo ""
    echo "This script requires either gnome-terminal or tmux."
    echo ""
    echo "Please install one:"
    echo "  For GNOME (recommended for Jetson):"
    echo "    sudo apt update && sudo apt install gnome-terminal"
    echo ""
    echo "  For tmux (more flexible):"
    echo "    sudo apt update && sudo apt install tmux"
    echo ""
    echo "════════════════════════════════════════════════════════════════"
    exit 1
fi
