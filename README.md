# ROVIO VIO Workspace for ROS2

ROS2 workspace for ROVIO Visual-Inertial Odometry with Intel RealSense D435i support.

## Features
- ROVIO: Robust Visual Inertial Odometry (ROS2 Humble)
- Intel RealSense D435i integration
- QoS relay node for RealSense-ROVIO compatibility
- Pre-configured for D435i camera intrinsics and IMU
- **ARM64/Jetson Orin optimized** - Auto-detection of CPU architecture

## Supported Platforms

| Platform | Architecture | OS | Status |
|----------|--------------|----|---------|
| Desktop/Laptop | x86_64 | Ubuntu 22.04 | ✅ Tested |
| **NVIDIA Jetson Orin** | **ARM64** | **Ubuntu 20.04/22.04 (JetPack 5.x+)** | **✅ Optimized** |
| NVIDIA Jetson Xavier | ARM64 | Ubuntu 20.04 | ⚠️ Experimental |

## Prerequisites

### Desktop/Laptop (x86_64)
- Ubuntu 22.04
- ROS2 Humble
- Intel RealSense SDK (librealsense2)
- OpenCV, Eigen3, yaml-cpp

### Jetson Orin/Xavier (ARM64)
- Ubuntu 20.04 or 22.04
- **JetPack 5.0+** (includes CUDA, OpenCV)
- ROS2 Humble
- Intel RealSense SDK for ARM64
- Minimum 8GB RAM (16GB recommended)
- **8GB swap space recommended** (if RAM < 16GB)

## Installation

### 1. Install ROS2 Humble
```bash
# Follow: https://docs.ros.org/en/humble/Installation.html
```

### 2. Install RealSense SDK

#### On Desktop/Laptop (x86_64)
```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/librealsense.list

sudo apt update
sudo apt install -y librealsense2-dkms librealsense2-utils librealsense2-dev
```

#### On Jetson Orin/Xavier (ARM64) - Build from Source

**Pre-built binaries have compatibility issues on ARM64. Build from source:**

```bash
# Install dependencies
sudo apt update
sudo apt install -y git cmake build-essential libssl-dev \
  libusb-1.0-0-dev libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

# Clone librealsense
cd ~
git clone --depth 1 --branch v2.56.2 https://github.com/IntelRealSense/librealsense.git
cd librealsense

# Build (takes 10-15 minutes)
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_EXAMPLES=false \
  -DBUILD_GRAPHICAL_EXAMPLES=false \
  -DBUILD_PYTHON_BINDINGS=false \
  -DFORCE_RSUSB_BACKEND=ON \
  -DBUILD_WITH_CUDA=OFF
make -j4

# Install
sudo make install
sudo ldconfig

# Add to bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc

# Verify
rs-enumerate-devices
```

### 3. Install RealSense ROS2 wrapper

#### On Desktop (x86_64)
```bash
sudo apt install -y ros-humble-realsense2-camera ros-humble-realsense2-camera-msgs
```

#### On Jetson (ARM64) - Build from Source
```bash
cd ~/rovio_ws/src
git clone --depth 1 --branch 4.56.1 https://github.com/IntelRealSense/realsense-ros.git

# Install dependencies
sudo apt install -y ros-humble-diagnostic-updater ros-humble-xacro

# Build
cd ~/rovio_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select realsense2_camera_msgs realsense2_description realsense2_camera \
  --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 4

source install/setup.bash
```

### 4. Clone and build workspace

#### On Desktop/Laptop (x86_64)
```bash
cd ~
git clone <your-repo-url> rovio_ws
cd rovio_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

#### On Jetson Orin/Xavier (ARM64)

**Step 1: Setup swap space (if RAM < 16GB)**
```bash
# Create 8GB swap file
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Make permanent
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

# Verify
free -h
```

**Step 2: Set MAXN power mode**
```bash
# Set to maximum performance
sudo nvpmodel -m 0
sudo jetson_clocks

# Verify
sudo nvpmodel -q
```

**Step 3: Clone and build**
```bash
cd ~
git clone <your-repo-url> rovio_ws
cd rovio_ws
source /opt/ros/humble/setup.bash

# Build with limited parallel jobs (prevents OOM)
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --parallel-workers 4

source install/setup.bash
```

**Build system automatically detects ARM64 and applies optimizations:**
- Uses `-mcpu=native -O3` instead of `-march=native`
- Enables NEON SIMD instructions
- You should see: `Detected ARM64 architecture - using ARM-specific optimizations`
Build bị interrupt. Hãy tiếp tục build với optimization thấp hơn (Debug mode) để giảm memory usage:
## Performance Tuning for Jetson

### Reduce Computational Load

For better real-time performance on Jetson, use the optimized config:

```bash
# Use Jetson-optimized config (640x480, 15 features, reduced params)
ros2 run rovio rovio_node --ros-args \
  -p filter_config:=$(ros2 pkg prefix rovio)/share/rovio/cfg/d435i_jetson_config.yaml
```

**Jetson config optimizations:**
- Image resolution: 640x480 (vs 1280x720)
- Max features: 15 (vs 25)
- Detector frequency: 3 Hz (vs 5 Hz)
- Patch size: 6 (vs 8)
- **Expected performance: 20-30 Hz on Jetson Orin Nano**

### Monitor Resources

```bash
# Check CPU/GPU/RAM/Temperature
sudo tegrastats

# Check ROVIO publishing rate
ros2 topic hz /rovio/odometry

# Check memory usage
free -h
```

### If ROVIO runs slowly (<15 Hz):

1. **Ensure MAXN mode**: `sudo nvpmodel -m 0 && sudo jetson_clocks`
2. **Reduce features further**: Edit `d435i_jetson_config.yaml` → `MaxFeatureCount: 10`
3. **Lower camera resolution**: 480x360 in RealSense launch
4. **Check temperature**: `cat /sys/devices/virtual/thermal/thermal_zone*/temp`
5. **Add active cooling** if temp > 80°C

## Usage

### 🚀 Quick Start for Jetson Orin

**See [JETSON_QUICKSTART.md](JETSON_QUICKSTART.md) for complete running instructions!**

3-Terminal Setup:
1. **Terminal 1**: Launch RealSense camera
2. **Terminal 2**: Run ROVIO node
3. **Terminal 3**: Monitor odometry output

Full commands and troubleshooting in the quickstart guide.

---

### Run with Intel RealSense D435i (Live)

**Terminal 1: RealSense Camera**
```bash
cd ~/rovio_ws && source install/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=false \
  enable_infra1:=true \
  enable_infra2:=false \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=1 \
  infra_width:=640 \
  infra_height:=480 \
  infra_fps:=30
```

**Terminal 2: ROVIO VIO**

```bash
cd ~/rovio_ws && source install/setup.bash

# Ensure MAXN mode (Jetson only)
sudo nvpmodel -m 0 && sudo jetson_clocks

# Run ROVIO
ros2 run rovio rovio_node --ros-args \
  -p filter_config:=$(ros2 pkg prefix rovio)/share/rovio/cfg/rovio.info \
  -p imu_topic:=/camera/camera/imu \
  -p cam0_topic:=/camera/camera/infra1/image_rect_raw
```

**Important**: Move camera slowly with rotation to initialize tracking!

**Terminal 3: View Position Output**
```bash
cd ~/rovio_ws && source install/setup.bash
ros2 topic echo /rovio/odometry --field pose.pose.position

# Or check publishing rate
ros2 topic hz /rovio/odometry
```

**Terminal 4 (Jetson only): Monitor Performance**
```bash
# Real-time stats (CPU, GPU, RAM, Temp)
sudo tegrastats

# Check publishing rate
ros2 topic hz /rovio/odometry
```

### Run with EUROC Dataset (Offline)

```bash
cd ~/rovio_ws
source install/setup.bash

# Download and convert EUROC dataset (example: MH_01_easy)
# ... (download and convert to ROS2 bag)

# Run rosbag loader
ros2 launch rovio ros2_rovio_rosbag_loader_launch.py \
  rosbag_filename:=$(pwd)/datasets/machine_hall/MH_01_easy/MH_01_easy_ros2/MH_01_easy_ros2.db3
```

## Configuration Files

- `src/rovio/cfg/d435i_config.yaml` - ROVIO config for RealSense D435i (1280x720, 25 features)
- `src/rovio/cfg/d435i_jetson_config.yaml` - **Jetson-optimized** (640x480, 15 features) ⭐
- `src/rovio/cfg/rovio.info` - Default ROVIO config (for EUROC dataset)
- `src/rovio/cfg/euroc_cam0.yaml` - EUROC dataset cam0
- `src/rovio/cfg/euroc_cam1.yaml` - EUROC dataset cam1

## Output Topics

- `/rovio/odometry` - Full 6DOF pose (position + orientation) with velocity
- `/rovio/pose_with_covariance_stamped` - Pose with uncertainty
- `/rovio/imu_biases` - Estimated IMU biases
- `/rovio/markers` - Tracked feature visualization
- `/rovio/pcl` - Point cloud of features
- `/rovio/patch` - Feature patch visualization

## Troubleshooting

### QoS Incompatibility Warning
If you see "incompatible QoS" warnings, ensure the QoS relay node is running between RealSense and ROVIO.

### IMU Device Busy
```bash
# Kill all RealSense processes
sudo pkill -9 -f realsense
# Wait 2-3 seconds, then restart
```

### ROVIO Not Initializing
- Move the camera slowly with rotation at start
- Ensure environment has sufficient visual features
- Check that both image and IMU topics are publishing

### Jetson-Specific Issues

**Build fails with "Killed" or memory errors:**
```bash
# Add/increase swap space
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile

# Rebuild with fewer parallel jobs
cd ~/rovio_ws
rm -rf build install log
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 2
```

**ROVIO runs slowly (<10 Hz):**
1. Check power mode: `sudo nvpmodel -q` (should be mode 0 - MAXN)
2. Set MAXN: `sudo nvpmodel -m 0 && sudo jetson_clocks`
3. Use Jetson config: `d435i_jetson_config.yaml`
4. Reduce MaxFeatureCount to 10-12 in config
5. Lower camera resolution to 480x360
6. Check temperature: `cat /sys/devices/virtual/thermal/thermal_zone*/temp`

**High CPU temperature (>80°C):**
- Add active cooling (fan)
- Reduce power mode: `sudo nvpmodel -m 1` (15W mode)
- Lower frame rate in RealSense launch

**RealSense not detected on Jetson:**
```bash
# Check USB connection (use USB 3.0 blue port)
lsusb | grep Intel

# Verify librealsense
rs-enumerate-devices

# If not found, reinstall for ARM64
sudo apt install librealsense2-utils librealsense2-dev
```

## Repository Structure

```
rovio_ws/
├── src/
│   ├── rovio/              # Main ROVIO package
│   │   ├── cfg/            # Configuration files
│   │   │   ├── d435i_config.yaml          # Desktop config
│   │   │   ├── d435i_jetson_config.yaml   # Jetson optimized ⭐
│   │   │   └── euroc_*.yaml               # EUROC dataset configs
│   │   ├── doc/            # Documentation
│   │   │   ├── JetsonSetup.md             # Detailed Jetson guide
│   │   │   └── CustomSetup.md             # Custom sensor setup
│   │   ├── launch/         # ROS2 launch files
│   │   └── scripts/        # Helper scripts
│   ├── kindr/              # Kinematics and dynamics library
│   └── rovio_interfaces/   # Custom ROS2 messages
├── tools/
│   └── dummy_pub.py        # Test publisher for smoke testing
└── .gitignore
```

## Platform-Specific Performance

### Jetson Orin Nano 8GB (MAXN mode)
- **Processing Rate**: 20-30 Hz
- **CPU Usage**: 40-50% (2 cores)
- **Memory**: 2-3 GB
- **Camera**: 640x480 @ 30 FPS
- **Features**: 15-20 tracked

### Desktop i7-8700K + RTX 2060
- **Processing Rate**: 30-60 Hz
- **CPU Usage**: 20-30% (2 cores)
- **Memory**: 1-2 GB
- **Camera**: 1280x720 @ 30 FPS
- **Features**: 20-25 tracked

## Additional Resources

- **[Jetson Setup Guide](src/rovio/doc/JetsonSetup.md)** - Detailed Jetson instructions
- **[Custom Sensor Setup](src/rovio/doc/CustomSetup.md)** - Camera-IMU calibration
- **[Vietnamese Guide](src/rovio/doc/JETSON_CHANGES_VI.md)** - Hướng dẫn tiếng Việt

## Credits

- Original ROVIO: https://github.com/ethz-asl/rovio
- ROS2 port: https://github.com/suyash023/rovio
- RealSense ROS: https://github.com/IntelRealSense/realsense-ros
- ARM64/Jetson optimizations: Community contributions

## License

See individual package licenses in `src/*/LICENSE` or `src/*/package.xml`.
