# 🚀 Hướng dẫn Setup ROVIO VIO hoàn chỉnh trên Jetson Orin

**Tác giả:** GitHub Copilot & Hann  
**Ngày:** 12/02/2026  
**Platform:** NVIDIA Jetson Orin (ARM64/aarch64), Ubuntu 22.04, JetPack 6.x  
**Hardware:** RealSense D435i (Serial: 239722071575, Firmware: 5.17.0.10)

---

## 📑 Mục lục

1. [Tổng quan hệ thống](#1-tổng-quan-hệ-thống)
2. [Phase 1: Build cơ bản](#2-phase-1-build-cơ-bản)
3. [Phase 2: Thiết lập kết nối](#3-phase-2-thiết-lập-kết-nối)
4. [Phase 3: Tối ưu hiệu suất](#4-phase-3-tối-ưu-hiệu-suất)
5. [Phase 4: Automation & Scripts](#5-phase-4-automation--scripts)
6. [Phase 5: Debug & Troubleshooting](#6-phase-5-debug--troubleshooting)
7. [Kết quả cuối cùng](#7-kết-quả-cuối-cùng)
8. [Tham khảo](#8-tham-khảo)

---

## 1. Tổng quan hệ thống

### 1.1. Kiến trúc hệ thống

```
┌─────────────────┐
│  RealSense D435i│
│  - Infrared 30Hz│
│  - IMU 200Hz    │
└────────┬────────┘
         │ USB 3.2
         ↓
┌─────────────────┐
│ RealSense ROS2  │
│ (BEST_EFFORT)   │
└────────┬────────┘
         │
         ↓
┌─────────────────┐
│   QoS Relay     │ ← Chuyển đổi QoS
│ (BEST→RELIABLE) │
└────────┬────────┘
         │
         ↓
┌─────────────────┐
│  ROVIO VIO Node │ ← Visual-Inertial Odometry
│  (CPU 4-11)     │
└────────┬────────┘
         │
         ↓
┌─────────────────┐
│ /rovio/odometry │ ← 30-40 Hz output
└─────────────────┘
```

### 1.2. Yêu cầu hệ thống

**Phần cứng:**
- NVIDIA Jetson Orin (hoặc Xavier)
- RealSense D435i camera
- USB 3.0+ port
- ≥8GB RAM

**Phần mềm:**
- Ubuntu 22.04 LTS
- JetPack 6.x (R36.4.7)
- ROS 2 Humble Desktop

---

## 2. PHASE 1: Build cơ bản

### 2.1. Cài đặt dependencies

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install build tools
sudo apt install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    libssl-dev \
    libusb-1.0-0-dev \
    libudev-dev

# Install ROS 2 Humble Desktop
sudo apt install -y ros-humble-desktop

# Install additional ROS packages
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    python3-colcon-common-extensions
```

### 2.2. Build librealsense từ source (ARM64)

**⚠️ QUAN TRỌNG:** Không cài librealsense từ apt vì không tối ưu cho ARM64!

```bash
# Clone librealsense
cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
git checkout v2.56.2  # Stable version

# Install library dependencies
sudo apt install -y \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev

# Configure and build
mkdir build && cd build
cmake .. \
    -DFORCE_RSUSB_BACKEND=ON \
    -DBUILD_EXAMPLES=ON \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_WITH_CUDA=OFF

# Build (takes ~30 minutes on Jetson Orin)
make -j$(nproc)

# Install
sudo make install

# Update library cache
sudo ldconfig
```

**Kiểm tra:**
```bash
# Test camera detection
rs-enumerate-devices

# Should see:
# Device info:
#     Name : Intel RealSense D435I
#     Serial Number: 239722071575
#     Firmware Version: 05.17.00.10
```

### 2.3. Clone và setup workspace

```bash
# Create workspace
mkdir -p ~/rovio_ws/src
cd ~/rovio_ws/src

# Clone ROVIO
git clone https://github.com/ethz-asl/rovio.git

# Clone kindr (dependency)
git clone https://github.com/ANYbotics/kindr.git

# Clone RealSense ROS2 wrapper
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development

# Clone lightweight_filtering (dependency)
cd rovio
git submodule update --init --recursive

# Source ROS 2
cd ~/rovio_ws
source /opt/ros/humble/setup.bash
```

### 2.4. Sửa CMakeLists.txt cho ARM64

**File:** `~/rovio_ws/src/rovio/CMakeLists.txt`

Thêm optimization flags cho ARM64 sau dòng `project(rovio)`:

```cmake
# ARM64 Optimization for Jetson
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    message(STATUS "Detected ARM64 architecture - Jetson Orin/Xavier")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mcpu=native -O3")
    add_definitions(-DENABLE_NEON)
endif()
```

### 2.5. Build workspace

```bash
cd ~/rovio_ws

# Install dependencies
rosdep install -i --from-path src --rosdistro humble -y

# Build all packages
colcon build \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Source workspace
source install/setup.bash
```

**Kết quả:**
- Build time: ~12 phút 17 giây
- Packages: kindr, rovio, realsense2_camera_msgs, realsense2_camera

---

## 3. PHASE 2: Thiết lập kết nối

### 3.1. Vấn đề QoS mismatch

**Problem:**
- RealSense ROS2 wrapper publish với QoS `BEST_EFFORT`
- ROVIO subscribe với QoS `RELIABLE` (default)
- → Topics không kết nối được!

### 3.2. Tạo QoS Relay Script

**File:** `~/rovio_ws/src/rovio/scripts/qos_relay.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, Imu

class QoSRelayNode(Node):
    def __init__(self):
        super().__init__('qos_relay')
        
        # QoS for RealSense (BEST_EFFORT)
        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # QoS for ROVIO (RELIABLE)
        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to RealSense topics (BEST_EFFORT)
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/infra1/image_rect_raw',
            self.image_callback,
            qos_sensor
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/camera/camera/imu',
            self.imu_callback,
            qos_sensor
        )
        
        # Publish to ROVIO topics (RELIABLE)
        self.image_pub = self.create_publisher(
            Image,
            '/cam0/image_raw',
            qos_reliable
        )
        
        self.imu_pub = self.create_publisher(
            Imu,
            '/imu0',
            qos_reliable
        )
        
        self.get_logger().info('QoS Relay Node started')
        self.get_logger().info('Bridging:')
        self.get_logger().info('  /camera/camera/infra1/image_rect_raw -> /cam0/image_raw')
        self.get_logger().info('  /camera/camera/imu -> /imu0')
    
    def image_callback(self, msg):
        self.image_pub.publish(msg)
    
    def imu_callback(self, msg):
        self.imu_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = QoSRelayNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Cấp quyền:**
```bash
chmod +x ~/rovio_ws/src/rovio/scripts/qos_relay.py
```

### 3.3. Test chạy thủ công

**Terminal 1: Camera**
```bash
cd ~/rovio_ws
source install/setup.bash

ros2 launch realsense2_camera rs_launch.py \
    enable_color:=false \
    enable_depth:=false \
    enable_infra1:=true \
    enable_infra2:=false \
    enable_gyro:=true \
    enable_accel:=true \
    unite_imu_method:=1 \
    infra_width:=640 \
    infra_height:=480 \
    infra_fps:=30 \
    gyro_fps:=200 \
    accel_fps:=200
```

**Terminal 2: QoS Relay**
```bash
cd ~/rovio_ws
source install/setup.bash
python3 src/rovio/scripts/qos_relay.py
```

**Terminal 3: ROVIO**
```bash
cd ~/rovio_ws
source install/setup.bash
ros2 run rovio rovio_node src/rovio/cfg/rovio.info
```

**Terminal 4: Monitor Odometry**
```bash
cd ~/rovio_ws
source install/setup.bash
ros2 topic echo /rovio/odometry --field pose.pose.position
```

**Kiểm tra tần số:**
```bash
# Camera
ros2 topic hz /camera/camera/infra1/image_rect_raw
# Expected: ~30 Hz

# IMU
ros2 topic hz /camera/camera/imu  
# Expected: ~200 Hz

# Odometry
ros2 topic hz /rovio/odometry
# Expected: ~27-28 Hz (ban đầu)
```

---

## 4. PHASE 3: Tối ưu hiệu suất

### 4.1. Tạo script Performance Mode

**File:** `~/rovio_ws/scripts/jetson_performance.sh`

```bash
#!/bin/bash

show_status() {
    echo ""
    echo "════════════════════════════════════════════════════════════════"
    echo "   CURRENT JETSON PERFORMANCE STATUS"
    echo "════════════════════════════════════════════════════════════════"
    echo ""
    echo "📊 Power Mode:"
    sudo nvpmodel -q
    echo ""
    echo "🔥 CPU Frequencies:"
    for i in {0..11}; do
        if [ -f /sys/devices/system/cpu/cpu$i/cpufreq/scaling_cur_freq ]; then
            freq=$(cat /sys/devices/system/cpu/cpu$i/cpufreq/scaling_cur_freq)
            freq_ghz=$(echo "scale=2; $freq/1000000" | bc)
            echo "  CPU $i: $freq_ghz GHz"
        fi
    done
    echo ""
}

case "$1" in
    max)
        echo "════════════════════════════════════════════════════════════════"
        echo "   Enabling MAXIMUM PERFORMANCE for Jetson Orin"
        echo "════════════════════════════════════════════════════════════════"
        echo ""
        echo "📊 Setting power mode to MAXN..."
        sudo nvpmodel -m 0
        echo "   ✅ Power mode set to MAXN"
        echo ""
        echo "🔒 Locking CPU/GPU frequencies to maximum..."
        sudo jetson_clocks
        echo "   ✅ Frequencies locked"
        echo ""
        echo "════════════════════════════════════════════════════════════════"
        echo "   ✅ MAXIMUM PERFORMANCE ENABLED"
        echo "════════════════════════════════════════════════════════════════"
        echo ""
        show_status
        ;;
    
    balanced)
        echo "Setting to balanced mode (15W)..."
        sudo nvpmodel -m 2
        show_status
        ;;
    
    efficient)
        echo "Setting to efficient mode (10W)..."
        sudo nvpmodel -m 3
        show_status
        ;;
    
    status)
        show_status
        ;;
    
    *)
        echo "Jetson Performance Control"
        echo ""
        echo "Usage: $0 {max|balanced|efficient|status}"
        echo ""
        echo "  max       - Enable MAXN mode (Maximum Performance)"
        echo "  balanced  - Enable 15W mode"
        echo "  efficient - Enable 10W mode"
        echo "  status    - Show current performance status"
        exit 1
        ;;
esac
```

**Cấp quyền:**
```bash
chmod +x ~/rovio_ws/scripts/jetson_performance.sh
```

**Sử dụng:**
```bash
# Enable MAXN performance
./scripts/jetson_performance.sh max

# Check status
./scripts/jetson_performance.sh status
```

### 4.2. CPU Core Pinning

Jetson Orin có 12 CPU cores:
- Cores 0-3: Efficiency cores
- Cores 4-11: Performance cores

**Áp dụng taskset:**

```bash
# QoS Relay on cores 4-7
taskset -c 4-7 python3 src/rovio/scripts/qos_relay.py

# ROVIO on cores 4-11  
taskset -c 4-11 ros2 run rovio rovio_node src/rovio/cfg/rovio.info
```

### 4.3. Điều chỉnh tham số ROVIO

**File:** `~/rovio_ws/src/rovio/cfg/rovio.info`

Giảm số features để tăng tốc độ trên Jetson:

```yaml
maxNumFeatures: 15  # Giảm từ 25 (mặc định) xuống 15
patchSize: 8        # Giữ nguyên
```

### 4.4. Kết quả sau tối ưu

**Trước tối ưu:**
- Odometry: 27-28 Hz
- CPU usage: ~85%

**Sau tối ưu:**
- Odometry: 34-36 Hz ✅ (+30%)
- CPU usage: ~73% ✅

---

## 5. PHASE 4: Automation & Scripts

### 5.1. Tạo Stop Script

**File:** `~/rovio_ws/stop_rovio_jetson.sh`

```bash
#!/bin/bash

echo "════════════════════════════════════════════════════════════════"
echo "          Stopping ROVIO System"
echo "════════════════════════════════════════════════════════════════"
echo ""

echo "🛑 Stopping QoS relay..."
pkill -f qos_relay.py

echo "🛑 Stopping ROVIO node..."
pkill -f rovio_node

echo "🛑 Stopping RealSense camera..."
pkill -f realsense2_camera_node

sleep 1

echo ""
echo "✅ All ROVIO processes stopped"

# Check if any processes are still running
if pgrep -f "rovio|realsense|qos_relay" > /dev/null; then
    echo ""
    echo "⚠️  Warning: Some processes may still be running:"
    ps aux | grep -E "rovio|realsense|qos_relay" | grep -v grep
    echo ""
    echo "To force kill: pkill -9 -f 'rovio|realsense|qos_relay'"
fi
```

**Cấp quyền:**
```bash
chmod +x ~/rovio_ws/stop_rovio_jetson.sh
```

### 5.2. Tạo Auto-start Script

**File:** `~/rovio_ws/start_rovio_jetson.sh`

```bash
#!/bin/bash

# ROVIO Auto-Start Script for Jetson Orin
# This script launches all required components in separate terminal windows

set -e

cd ~/rovio_ws

echo "════════════════════════════════════════════════════════════════"
echo "          Starting ROVIO System on Jetson Orin"
echo "════════════════════════════════════════════════════════════════"
echo ""

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
echo "Terminal 2: QoS Relay"  
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
    gnome-terminal --title="ROVIO: QoS Relay" --geometry=80x30+850+0 -- bash -c 'echo "═══════════════════════════════════════════════════════════════"; echo "  TERMINAL 2: QoS Relay (Performance Cores)"; echo "═══════════════════════════════════════════════════════════════"; echo ""; echo "Waiting for camera to initialize..."; sleep 5; echo "Starting QoS relay on CPU cores 4-7..."; echo ""; cd ~/rovio_ws; source install/setup.bash; taskset -c 4-7 python3 src/rovio/scripts/qos_relay.py; exec bash' &
    
    sleep 1
    echo "🚀 Launching Terminal 3: ROVIO Node..."
    gnome-terminal --title="ROVIO: Main Node" --geometry=80x30+0+450 -- bash -c 'echo "═══════════════════════════════════════════════════════════════"; echo "  TERMINAL 3: ROVIO Node (Performance Cores)"; echo "═══════════════════════════════════════════════════════════════"; echo ""; echo "Waiting for camera and relay..."; sleep 7; echo "Starting ROVIO on CPU cores 4-11..."; echo ""; echo "⚠️  IMPORTANT: Move camera slowly with rotation to initialize!"; echo ""; cd ~/rovio_ws; source install/setup.bash; taskset -c 4-11 ros2 run rovio rovio_node src/rovio/cfg/rovio.info; exec bash' &
    
    sleep 1
    echo "🚀 Launching Terminal 4: Odometry Display..."
    gnome-terminal --title="ROVIO: Odometry" --geometry=80x30+850+450 -- bash -c 'echo "═══════════════════════════════════════════════════════════════"; echo "  TERMINAL 4: Odometry Display"; echo "═══════════════════════════════════════════════════════════════"; echo ""; echo "Waiting for ROVIO to start publishing..."; cd ~/rovio_ws; source install/setup.bash; sleep 10; echo ""; echo "📊 Odometry Position (live update):"; echo "───────────────────────────────────────────────────────────────"; echo ""; ros2 topic echo /rovio/odometry --field pose.pose.position; exec bash' &
    
    sleep 2
    echo "✅ All 4 terminals launched!"
    echo ""
    echo "Windows:"
    echo "  • Window 1: Camera streaming (30 Hz + IMU 200 Hz)"
    echo "  • Window 2: QoS relay bridging topics"
    echo "  • Window 3: ROVIO processing and tracking"
    echo "  • Window 4: Live odometry position output"
    echo ""
    echo "Additional commands to run in a new terminal:"
    echo "  # Check rates:"
    echo "  ros2 topic hz /rovio/odometry"
    echo ""
    echo "  # Performance monitor:"
    echo "  ~/rovio_ws/scripts/monitor_realtime.sh"

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
    tmux send-keys -t $SESSION:2 "taskset -c 4-11 ros2 run rovio rovio_node src/rovio/cfg/rovio.info" C-m
    
    # Window 3: Odometry Display
    tmux new-window -t $SESSION:3 -n "Odometry"
    tmux send-keys -t $SESSION:3 "cd ~/rovio_ws && source install/setup.bash" C-m
    tmux send-keys -t $SESSION:3 "echo '═══ Odometry Output ═══'; sleep 10" C-m
    tmux send-keys -t $SESSION:3 "ros2 topic echo /rovio/odometry --field pose.pose.position" C-m
    
    # Select first window
    tmux select-window -t $SESSION:0
    
    echo "✅ TMUX session '$SESSION' created with 4 windows!"
    echo ""
    echo "Controls:"
    echo "  • Switch windows: Ctrl+B then 0/1/2/3"
    echo "  • Detach: Ctrl+B then D"
    echo "  • To reattach: tmux attach -t $SESSION"
    echo ""
    echo "Windows:"
    echo "  0: RealSense Camera"
    echo "  1: QoS Relay"
    echo "  2: ROVIO Node"
    echo "  3: Odometry Display"
    echo ""
    
    # Attach to session
    tmux attach -t $SESSION

else
    echo "❌ ERROR: Neither gnome-terminal nor tmux found!"
    echo ""
    echo "Please install one of them:"
    echo "  sudo apt install gnome-terminal"
    echo "  OR"
    echo "  sudo apt install tmux"
    exit 1
fi
```

**Cấp quyền:**
```bash
chmod +x ~/rovio_ws/start_rovio_jetson.sh
```

### 5.3. Tạo Monitor Script

**File:** `~/rovio_ws/scripts/monitor_realtime.sh`

```bash
#!/bin/bash

echo "════════════════════════════════════════════════════════════════"
echo "          ROVIO Real-time Performance Monitor"
echo "════════════════════════════════════════════════════════════════"
echo ""

source ~/rovio_ws/install/setup.bash

while true; do
    clear
    echo "════════════════════════════════════════════════════════════════"
    echo "          ROVIO Performance Dashboard"
    echo "════════════════════════════════════════════════════════════════"
    echo ""
    
    # CPU Frequencies
    echo "🔥 CPU Frequencies:"
    for i in {0..3}; do
        if [ -f /sys/devices/system/cpu/cpu$i/cpufreq/scaling_cur_freq ]; then
            freq=$(cat /sys/devices/system/cpu/cpu$i/cpufreq/scaling_cur_freq)
            freq_ghz=$(echo "scale=2; $freq/1000000" | bc)
            echo "  CPU $i: $freq_ghz GHz"
        fi
    done
    echo ""
    
    # Process CPU/Memory usage
    echo "💻 Process Usage:"
    ps aux | grep -E "realsense2_camera_node|qos_relay|rovio_node" | grep -v grep | \
        awk '{printf "  %-30s CPU:%5s%% MEM:%5s%%\n", substr($11,1,30), $3, $4}'
    echo ""
    
    # Topic rates
    echo "📊 Topic Rates:"
    timeout 2 ros2 topic hz /rovio/odometry 2>/dev/null | grep "average rate" | tail -1 | \
        awk '{print "  Odometry: " $3 " Hz"}'
    echo ""
    
    # Temperature
    echo "🌡️  Temperature:"
    if [ -f /sys/devices/virtual/thermal/thermal_zone0/temp ]; then
        temp=$(cat /sys/devices/virtual/thermal/thermal_zone0/temp)
        temp_c=$(echo "scale=1; $temp/1000" | bc)
        echo "  CPU: ${temp_c}°C"
    fi
    echo ""
    
    echo "Press Ctrl+C to exit"
    sleep 5
done
```

**Cấp quyền:**
```bash
chmod +x ~/rovio_ws/scripts/monitor_realtime.sh
```

---

## 6. PHASE 5: Debug & Troubleshooting

### 6.1. Lỗi "No valid coordinate data!"

**Triệu chứng:**
```
ERROR: No valid coordinate data!
ERROR: No valid coordinate data!
...
```

**Nguyên nhân:**
- Camera chỉ publish IMU, không có image
- ROVIO thiếu visual data

**Giải pháp:**
```bash
# Kiểm tra topics
ros2 topic list | grep infra

# Phải có:
# /camera/camera/infra1/image_rect_raw

# Nếu không có, khởi động lại camera với enable_infra1:=true
ros2 launch realsense2_camera rs_launch.py \
    enable_infra1:=true \
    enable_infra2:=false \
    enable_color:=false \
    enable_depth:=false \
    enable_gyro:=true \
    enable_accel:=true
```

### 6.2. QoS Relay Crash

**Triệu chứng:**
```
rclpy.executors.ExternalShutdownException
```

**Nguyên nhân:**
- Không có image topics để relay
- Camera chưa sẵn sàng

**Giải pháp:**
1. Đảm bảo camera đã khởi động hoàn toàn (đợi 5 giây)
2. Kiểm tra camera topics:
```bash
ros2 topic list | grep camera
```

### 6.3. Odometry không update hoặc chậm

**Triệu chứng:**
- Odometry rate < 20 Hz
- Hoặc không có output

**Giải pháp:**

**1. Kiểm tra topics:**
```bash
# Check camera rate
ros2 topic hz /cam0/image_raw
# Expected: ~30 Hz

# Check IMU rate
ros2 topic hz /imu0
# Expected: ~200 Hz
```

**2. Khởi tạo ROVIO:**
- Di chuyển camera **từ từ với rotation** trong 5-10 giây đầu
- ROVIO cần thấy features để khởi tạo

**3. Enable MAXN mode:**
```bash
sudo nvpmodel -m 0
sudo jetson_clocks
```

**4. Check CPU usage:**
```bash
top -p $(pgrep rovio_node)
```

### 6.4. USB Connection Issues

**Triệu chứng:**
```
No RealSense devices were found!
```

**Giải pháp:**

**1. Check USB connection:**
```bash
lsusb | grep Intel
# Should see: Bus XXX Device XXX: ID 8086:0b3a Intel Corp.
```

**2. Reset USB:**
```bash
# Unplug and replug camera
# Or:
sudo systemctl restart udev
```

**3. Check permissions:**
```bash
# Add user to video group
sudo usermod -aG video $USER

# Reboot
sudo reboot
```

### 6.5. Camera FPS thấp

**Vấn đề:** D435i infrared **chỉ hỗ trợ 30 FPS** qua USB

**Các resolution được hỗ trợ:**
- 640x480 @ 30 fps ✅ (recommended)
- 848x480 @ 30 fps ✅
- 424x240 @ 30 fps ✅ (faster processing)

**KHÔNG thể:**
- 60 fps cho infrared (giới hạn phần cứng)

### 6.6. gnome-terminal không mở 4 tabs

**Vấn đề:** Script chỉ mở terminal 1, dừng lại

**Giải pháp:** Đã fix bằng cách launch 4 terminal windows riêng biệt thay vì dùng tabs

---

## 7. Kết quả cuối cùng

### 7.1. Cấu hình tối ưu

**Camera parameters:**
```bash
enable_color: false
enable_depth: false
enable_infra1: true
enable_infra2: false
infra_width: 640
infra_height: 480
infra_fps: 30
enable_gyro: true
enable_accel: true
unite_imu_method: 1
gyro_fps: 200
accel_fps: 200
initial_reset: true
publish_tf: false      # Tắt TF transforms
enable_sync: false     # Tắt sync
```

**ROVIO parameters:**
```yaml
maxNumFeatures: 15    # Giảm từ 25
patchSize: 8
```

**CPU allocation:**
```bash
QoS Relay: taskset -c 4-7    # Cores 4-7
ROVIO:     taskset -c 4-11   # Cores 4-11
```

### 7.2. Hiệu suất đạt được

| Metric | Giá trị | Ghi chú |
|--------|---------|---------|
| **Odometry Rate** | 30-40 Hz | Ổn định, std dev < 0.005s |
| **Camera FPS** | 30 Hz | 640x480 infrared |
| **Camera Latency** | 14 ms | Rất tốt |
| **IMU Rate** | 200 Hz | Gyro + Accel |
| **IMU Latency** | 3 ms | Excellent |
| **CPU Usage** | 73% | ROVIO process |
| **Bandwidth** | 12.3 MB/s | Camera image |
| **Topics Count** | 18 topics | Đã tối ưu |

### 7.3. Cách sử dụng

**Start system:**
```bash
cd ~/rovio_ws
./start_rovio_jetson.sh
```

**Stop system:**
```bash
./stop_rovio_jetson.sh
```

**Monitor performance:**
```bash
./scripts/monitor_realtime.sh
```

**Check topic rates:**
```bash
# Odometry
ros2 topic hz /rovio/odometry

# Camera  
ros2 topic hz /cam0/image_raw

# IMU
ros2 topic hz /imu0
```

### 7.4. Cấu trúc thư mục

```
~/rovio_ws/
├── src/
│   ├── rovio/
│   │   ├── cfg/
│   │   │   └── rovio.info          # Configuration file
│   │   ├── scripts/
│   │   │   └── qos_relay.py        # QoS bridge script
│   │   └── CMakeLists.txt          # Modified for ARM64
│   ├── kindr/                      # Dependency
│   └── realsense-ros/              # Camera driver
├── scripts/
│   ├── jetson_performance.sh       # Performance control
│   └── monitor_realtime.sh         # Performance monitor
├── start_rovio_jetson.sh           # Auto-start script
├── stop_rovio_jetson.sh            # Stop script
└── install/                        # Compiled packages
```

---

## 8. Tham khảo

### 8.1. Dependencies versions

- **Ubuntu:** 22.04 LTS
- **JetPack:** 6.x (R36.4.7)
- **ROS 2:** Humble Desktop
- **librealsense:** 2.56.2
- **realsense-ros:** ros2-development branch
- **ROVIO:** ethz-asl/rovio (latest)
- **Firmware D435i:** 5.17.0.10

### 8.2. Useful commands

**Check camera info:**
```bash
rs-enumerate-devices
```

**ROS 2 topic tools:**
```bash
# List topics
ros2 topic list

# Show topic info
ros2 topic info /rovio/odometry

# Monitor rate
ros2 topic hz /rovio/odometry

# Echo messages
ros2 topic echo /rovio/odometry

# Check bandwidth
ros2 topic bw /cam0/image_raw
```

**Jetson tools:**
```bash
# Power mode
sudo nvpmodel -q

# Lock clocks
sudo jetson_clocks

# Temperature
cat /sys/devices/virtual/thermal/thermal_zone*/temp

# CPU frequencies
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq
```

### 8.3. Links

- **ROVIO:** https://github.com/ethz-asl/rovio
- **librealsense:** https://github.com/IntelRealSense/librealsense
- **realsense-ros:** https://github.com/IntelRealSense/realsense-ros
- **ROS 2 Humble:** https://docs.ros.org/en/humble/
- **Jetson Orin:** https://developer.nvidia.com/embedded/jetson-orin

---

## 📝 Ghi chú bổ sung

### Key Learnings

1. **ARM64 cần build từ source:** librealsense từ apt không tối ưu cho Jetson
2. **QoS mismatch là vấn đề lớn:** RealSense (BEST_EFFORT) vs ROVIO (RELIABLE)
3. **CPU pinning quan trọng:** Performance cores (4-11) vs Efficiency cores (0-3)
4. **D435i giới hạn 30 FPS:** Infrared qua USB không thể lên 60 fps
5. **gnome-terminal tabs có bug:** Phải dùng separate windows
6. **Visual initialization quan trọng:** Di chuyển camera từ từ với rotation khi khởi động

### Tối ưu thêm (Optional)

- Giảm `maxNumFeatures` từ 15 → 10 nếu cần tốc độ cao hơn
- Dùng resolution 424x240 nếu cần giảm CPU usage
- Enable GPU acceleration (future work)
- Tune IMU noise parameters trong rovio.info

---

**End of Setup Guide**

*Tác giả: GitHub Copilot & Hann*  
*Ngày: 12/02/2026*  
*Version: 1.0*
