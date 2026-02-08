# ROVIO VIO Workspace for ROS2

ROS2 workspace for ROVIO Visual-Inertial Odometry with Intel RealSense D435i support.

## Features
- ROVIO: Robust Visual Inertial Odometry (ROS2 Humble)
- Intel RealSense D435i integration
- QoS relay node for RealSense-ROVIO compatibility
- Pre-configured for D435i camera intrinsics and IMU

## Prerequisites
- Ubuntu 22.04
- ROS2 Humble
- Intel RealSense SDK (librealsense2)
- OpenCV, Eigen3, yaml-cpp

## Installation

### 1. Install ROS2 Humble
```bash
# Follow: https://docs.ros.org/en/humble/Installation.html
```

### 2. Install RealSense SDK
```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/librealsense.list

sudo apt update
sudo apt install -y librealsense2-dkms librealsense2-utils librealsense2-dev
```

### 3. Install RealSense ROS2 wrapper
```bash
sudo apt install -y ros-humble-realsense2-camera ros-humble-realsense2-camera-msgs
```

### 4. Clone and build workspace
```bash
cd ~
git clone <your-repo-url> rovio_ws
cd rovio_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Usage

### Run with Intel RealSense D435i (Live)

**Terminal 1: RealSense Camera**
```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=false \
  enable_infra1:=true \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=1
```

**Terminal 2: QoS Relay (Best Effort → Reliable)**
```bash
cd ~/rovio_ws
source install/setup.bash
python3 src/rovio/scripts/qos_relay.py
```

**Terminal 3: ROVIO VIO**
```bash
cd ~/rovio_ws
source install/setup.bash
ros2 run rovio rovio_node --ros-args \
  -p filter_config:=$(pwd)/src/rovio/cfg/d435i_config.yaml
```

**Terminal 4: View Position Output**
```bash
source ~/rovio_ws/install/setup.bash
ros2 topic echo --field pose.pose.position /rovio/odometry
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

- `src/rovio/cfg/d435i_config.yaml` - ROVIO config for RealSense D435i
- `src/rovio/cfg/rovio.info` - Default ROVIO config (for EUROC dataset)

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

## Repository Structure

```
rovio_ws/
├── src/
│   ├── rovio/              # Main ROVIO package
│   ├── kindr/              # Kinematics and dynamics library
│   └── rovio_interfaces/   # Custom ROS2 messages
├── tools/
│   └── dummy_pub.py        # Test publisher for smoke testing
└── .gitignore
```

## Credits

- Original ROVIO: https://github.com/ethz-asl/rovio
- ROS2 port: https://github.com/suyash023/rovio
- RealSense ROS: https://github.com/IntelRealSense/realsense-ros

## License

See individual package licenses in `src/*/LICENSE` or `src/*/package.xml`.
