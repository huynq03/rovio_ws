# ROVIO on NVIDIA Jetson Orin / Xavier (ARM64)

This guide provides detailed instructions for building and running ROVIO on NVIDIA Jetson platforms.

## Prerequisites

### Hardware Requirements
- **Jetson Orin Nano/NX/AGX** or **Jetson Xavier NX/AGX**
- Minimum 8GB RAM (16GB recommended for full feature tracking)
- 32GB+ storage (for workspace and datasets)
- Camera with IMU (e.g., Intel RealSense D435i, T265)

### Software Requirements
- **JetPack 5.0+** (Ubuntu 20.04 or 22.04)
- **ROS 2 Humble** (desktop or base + additional packages)
- **OpenCV** (usually included with JetPack)
- **Eigen3**

## Installation Steps

### 1. Verify JetPack Installation
```bash
sudo apt-cache show nvidia-jetpack
# Should show version 5.x or higher
```

### 2. Install ROS 2 Humble
If not already installed:
```bash
# Add ROS 2 repository
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop
```

### 3. Install Dependencies
```bash
# Essential build tools
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip \
  git \
  cmake \
  libeigen3-dev \
  libyaml-cpp-dev

# Initialize rosdep (if not done already)
sudo rosdep init
rosdep update
```

### 4. Setup Swap Space (Recommended for <16GB RAM)
```bash
# Create 8GB swap file
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Make permanent (add to /etc/fstab)
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

# Verify
free -h
```

### 5. Clone and Build ROVIO
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Create workspace
mkdir -p ~/rovio_ws/src
cd ~/rovio_ws/src

# Clone repositories
git clone git@github.com:suyash023/rovio.git
git clone git@github.com:suyash023/rovio_interfaces.git
cd rovio
git submodule update --init --recursive

# Source helper commands
source ~/rovio_ws/src/rovio/scripts/rovio_commands.sh

# Build for Jetson (limited parallel jobs to avoid OOM)
cd ~/rovio_ws
build_rovio_jetson
```

## Performance Optimization

### Reduce Computational Load
For better real-time performance on Jetson, you can reduce the feature count and pyramid levels:

Edit `~/rovio_ws/src/rovio/CMakeLists.txt`:
```cmake
set(ROVIO_NMAXFEATURE 15 CACHE STRING "Number of features")  # Reduced from 25
set(ROVIO_NLEVELS 3 CACHE STRING "Number of pyramid levels") # Reduced from 4
set(ROVIO_PATCHSIZE 4 CACHE STRING "Patch size")             # Reduced from 6
```

Then rebuild:
```bash
cd ~/rovio_ws
rm -rf build install log
build_rovio_jetson
```

Or pass parameters during build:
```bash
colcon build --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DROVIO_NMAXFEATURE=15 \
  -DROVIO_NLEVELS=3 \
  -DROVIO_PATCHSIZE=4 \
  --parallel-workers 4
```

### Power Mode Configuration
For best performance, set Jetson to maximum power mode:
```bash
# Set to MAXN mode
sudo nvpmodel -m 0

# Maximize clock speeds
sudo jetson_clocks

# Verify current mode
sudo nvpmodel -q
```

Available power modes (use `sudo nvpmodel -m <mode>`):
- **Mode 0**: MAXN (all cores, max frequency)
- **Mode 1**: 15W (balanced)
- **Mode 2**: 30W (Orin NX)

### Reduce Image Resolution
If using high-resolution cameras (e.g., 1920x1080), consider downscaling in the config file:

Edit camera config (e.g., `cfg/d435i_config.yaml`):
```yaml
img_height: 480  # Reduced from 720
img_width: 640   # Reduced from 1280
```

## Running ROVIO on Jetson

### With RealSense D435i
```bash
# Terminal 1: Source workspace
cd ~/rovio_ws
source install/setup.bash

# Launch ROVIO with D435i config
ros2 launch rovio ros2_d435i_rovio_launch.py
```

### With Custom Camera-IMU Setup
See [CustomSetup.md](CustomSetup.md) for sensor calibration and configuration.

### Monitor Resource Usage
```bash
# Check CPU/GPU usage
sudo tegrastats

# Check memory
free -h

# Check temperature
cat /sys/devices/virtual/thermal/thermal_zone*/temp
```

## Troubleshooting

### Build Fails with "Killed" or Memory Errors
- **Solution**: Add/increase swap space and reduce parallel build jobs:
  ```bash
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 2
  ```

### ROVIO Runs Slowly (<10 Hz)
- Reduce `ROVIO_NMAXFEATURE` to 10-15
- Reduce image resolution in camera config
- Ensure Jetson is in MAXN mode (`sudo nvpmodel -m 0`)
- Lower `ROVIO_NLEVELS` to 2 or 3

### Camera Not Detected
- Check USB 3.0 connection (blue port)
- Verify camera driver is installed:
  ```bash
  # For RealSense
  realsense-viewer
  ```

### High CPU Temperature (>80°C)
- Add active cooling (fan)
- Reduce power mode: `sudo nvpmodel -m 1`
- Lower frame rate in camera config

## Benchmark Performance

Typical performance on **Jetson Orin Nano 8GB** (MAXN mode):
- **ROVIO Node**: 15-30 Hz (depends on features and image size)
- **Camera Resolution**: 640x480 @ 30 FPS
- **Features Tracked**: 15-20
- **CPU Usage**: 40-60% (2 cores)
- **Memory**: 2-3 GB

## Additional Resources
- [ROVIO Main README](../README.md)
- [Custom Sensor Setup](CustomSetup.md)
- [Jetson Developer Guide](https://developer.nvidia.com/embedded/develop/get-started-jetson)
- [ROS 2 on Jetson](https://nvidia-ai-iot.github.io/ros2_jetson/)
