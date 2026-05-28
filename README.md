# README - Chạy ROVIO với Intel RealSense D435i trên Jetson Orin Headless

## 1. Mục tiêu

Dựng lại pipeline chạy **ROVIO visual-inertial odometry** từ đầu với camera **Intel RealSense D435i** trên Jetson Orin, chạy qua SSH/headless, không cần GUI/RViz.

Pipeline cuối cùng:

```text
Intel RealSense D435i
  ├── infra1 image 30 Hz
  ├── infra2 image 30 Hz
  └── combined imu 200 Hz
        ↓
realsense2_camera
        ↓
rovio_qos_relay
        ↓
/imu0
/cam0/image_raw
/cam1/image_raw
        ↓
rovio_node
        ↓
/rovio/odometry
/rovio/health
/rovio/pose_with_covariance_stamped
/rovio/pcl
/rovio/markers
```

---

## 2. Môi trường đã dùng

```text
OS: Ubuntu 22.04 Jammy
ROS: ROS2 Humble
Machine: Jetson Orin / aarch64
Camera: Intel RealSense D435i
Workspace: ~/rovio_ws
```

Kiểm tra ban đầu:

```bash
echo "===== SYSTEM ====="
uname -a
lsb_release -a 2>/dev/null || cat /etc/os-release

echo
echo "===== ROS ====="
which ros2 || true
ros2 --version 2>/dev/null || true
printenv ROS_DISTRO || true

echo
echo "===== D435i USB ====="
lsusb | grep -i -E "intel|realsense|8086" || echo "NO REALSENSE FOUND IN lsusb"

echo
echo "===== REALSENSE ROS PKG ====="
ros2 pkg list | grep -E "realsense2_camera|realsense2_camera_msgs" || echo "NO realsense ROS2 pkg"
```

Kết quả mong muốn:

```text
ROS_DISTRO = humble
D435i xuất hiện trong lsusb
Có package realsense2_camera
```

---

## 3. Tạo workspace và clone source

```bash
mkdir -p ~/rovio_ws/src
cd ~/rovio_ws/src

git clone https://github.com/suyash023/rovio.git
git clone https://github.com/suyash023/rovio_interfaces.git
```

Kiểm tra source:

```bash
find ~/rovio_ws/src -maxdepth 2 -type d | sort
```

Kỳ vọng có:

```text
/home/hann/rovio_ws/src/rovio
/home/hann/rovio_ws/src/rovio_interfaces
```

---

## 4. Build ROVIO

```bash
cd ~/rovio_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Kết quả đã pass:

```text
rovio_interfaces: PASS
rovio: PASS
```

Kiểm tra executable:

```bash
cd ~/rovio_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 pkg list | grep -E "^rovio$|^rovio_interfaces$"
ros2 pkg executables rovio
```

Kỳ vọng:

```text
rovio feature_tracker_node
rovio rovio_node
rovio rovio_rosbag_loader
```

---

## 5. Test D435i headless

Chạy RealSense với infra1 + infra2 + IMU:

```bash
cd ~/rovio_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  enable_color:=false \
  enable_depth:=false \
  enable_infra1:=true \
  enable_infra2:=true \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=2
```

Ở terminal SSH khác, kiểm tra topic:

```bash
source /opt/ros/humble/setup.bash

ros2 topic list | grep -E "infra1|infra2|imu|gyro|accel"
ros2 topic hz /camera/camera/infra1/image_rect_raw
ros2 topic hz /camera/camera/infra2/image_rect_raw
ros2 topic hz /camera/camera/imu
```

Kết quả đã đạt:

```text
/camera/camera/infra1/image_rect_raw  ~30 Hz
/camera/camera/infra2/image_rect_raw  ~30 Hz
/camera/camera/imu                    ~200 Hz
```

---

## 6. Lỗi RealSense device busy và cách xử lý

Có lúc RealSense bị lỗi:

```text
xioctl(VIDIOC_S_FMT) failed, errno=16
Device or resource busy
```

Nguyên nhân: còn process `realsense2_camera_node` cũ giữ `/dev/video*`.

Kiểm tra:

```bash
ps -eo pid,ppid,stat,cmd | grep -E '[r]ealsense2_camera|[r]s_launch|[r]os2 launch realsense'

for d in /dev/video*; do
  echo "--- $d"
  fuser -v "$d" 2>/dev/null || true
done
```

Kill theo PID cụ thể, ví dụ:

```bash
kill <PID1> <PID2> 2>/dev/null || true
```

Sau đó kiểm tra lại:

```bash
ps -eo pid,ppid,stat,cmd | grep -E '[r]ealsense2_camera|[r]s_launch|[r]os2 launch realsense' || echo "NO REALSENSE PROCESS"
```

---

## 7. Lấy camera intrinsics từ D435i

Chạy RealSense rồi lấy `/camera_info`:

```bash
cd ~/rovio_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  enable_color:=false \
  enable_depth:=false \
  enable_infra1:=true \
  enable_infra2:=true \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=2
```

Ở terminal khác:

```bash
source /opt/ros/humble/setup.bash

ros2 topic echo --once /camera/camera/infra1/camera_info > /tmp/d435i_infra1_camera_info.yaml
ros2 topic echo --once /camera/camera/infra2/camera_info > /tmp/d435i_infra2_camera_info.yaml

sed -n '1,120p' /tmp/d435i_infra1_camera_info.yaml
sed -n '1,120p' /tmp/d435i_infra2_camera_info.yaml
```

Thông số đã lấy được:

```text
image_width: 848
image_height: 480

fx = 427.74517822265625
fy = 427.74517822265625
cx = 427.1524963378906
cy = 235.96437072753906

distortion_model: plumb_bob
distortion = [0, 0, 0, 0, 0]
```

Infra2 có:

```text
P[3] = -21.416364669799805
baseline ≈ 21.416364669799805 / 427.74517822265625
baseline ≈ 0.050068 m
```

---

## 8. Tạo camera config cho ROVIO

Tạo `d435i_infra1.yaml`:

```bash
cd ~/rovio_ws

cat > src/rovio/cfg/d435i_infra1.yaml <<'EOF'
###### Camera Calibration File For D435i Infra 1 ######
image_width: 848
image_height: 480
camera_name: d435i_infra1
camera_matrix:
  rows: 3
  cols: 3
  data: [427.74517822265625, 0.0, 427.1524963378906,
         0.0, 427.74517822265625, 235.96437072753906,
         0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.0, 0.0, 0.0, 0.0, 0.0]
EOF
```

Tạo `d435i_infra2.yaml`:

```bash
cd ~/rovio_ws

cat > src/rovio/cfg/d435i_infra2.yaml <<'EOF'
###### Camera Calibration File For D435i Infra 2 ######
image_width: 848
image_height: 480
camera_name: d435i_infra2
camera_matrix:
  rows: 3
  cols: 3
  data: [427.74517822265625, 0.0, 427.1524963378906,
         0.0, 427.74517822265625, 235.96437072753906,
         0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.0, 0.0, 0.0, 0.0, 0.0]
EOF
```

Kiểm tra:

```bash
ls -lah src/rovio/cfg/d435i_infra*.yaml
cat src/rovio/cfg/d435i_infra1.yaml
cat src/rovio/cfg/d435i_infra2.yaml
```

---

## 9. Lấy TF/static transform của D435i

Chạy RealSense với TF:

```bash
cd ~/rovio_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  enable_color:=false \
  enable_depth:=false \
  enable_infra1:=true \
  enable_infra2:=true \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=2 \
  publish_tf:=true
```

Ở terminal khác:

```bash
source /opt/ros/humble/setup.bash

timeout 5 ros2 topic echo --once /tf_static > /tmp/d435i_tf_static.yaml

grep -nE "frame_id:|child_frame_id:|translation:|rotation:|x:|y:|z:|w:" /tmp/d435i_tf_static.yaml | head -240
```

Thông tin quan trọng đã lấy được:

```text
camera_link -> camera_infra1_frame:
  translation: [0, 0, 0]
  rotation: identity

camera_link -> camera_infra2_frame:
  translation:
    x: 0.0
    y: -0.05006804317235947
    z: 0.0
  rotation: identity

camera_link -> camera_accel_frame:
  translation:
    x: -0.011739999987185001
    y: -0.005520000122487545
    z:  0.005100000184029341

camera_link -> camera_gyro_frame:
  translation:
    x: -0.011739999987185001
    y: -0.005520000122487545
    z:  0.005100000184029341
```

Kiểm tra frame thật của IMU:

```bash
ros2 topic echo --once /camera/camera/imu | sed -n "1,35p"
```

Kết quả quan trọng:

```text
/camera/camera/imu frame_id = camera_imu_optical_frame
```

Camera image frame:

```text
/camera/camera/infra1/camera_info frame_id = camera_infra1_optical_frame
```

Kết luận quan trọng:

```text
IMU message đã nằm trong optical frame.
Vì vậy qCM trong ROVIO phải để identity:
qCM = [0, 0, 0, 1]
```

Không dùng:

```text
qCM = [-0.5, 0.5, -0.5, 0.5]
```

vì nó sẽ xoay thừa và làm ROVIO divergence.

---

## 10. Tạo `rovio_d435i.info`

Copy từ config gốc:

```bash
cd ~/rovio_ws
cp src/rovio/cfg/rovio.info src/rovio/cfg/rovio_d435i.info
```

Patch Camera0/Camera1:

```bash
cd ~/rovio_ws

python3 - <<'PY'
from pathlib import Path

p = Path("src/rovio/cfg/rovio_d435i.info")
text = p.read_text()

def patch_block(text, name, vals):
    start = text.index(name)
    next_names = ["Camera1", "Init", "ImgUpdate", "PoseUpdate"]
    ends = [text.find(n, start + len(name)) for n in next_names if text.find(n, start + len(name)) != -1]
    end = min(ends) if ends else len(text)
    block = text[start:end]

    lines = block.splitlines()
    out = []
    for line in lines:
        stripped = line.strip()
        replaced = False
        for k, v in vals.items():
            if stripped.startswith(k + " "):
                indent = line[:len(line) - len(line.lstrip())]
                comment = ""
                if ";" in line:
                    comment = ";" + line.split(";", 1)[1]
                else:
                    comment = ";"
                out.append(f"{indent}{k} {v}{comment}")
                replaced = True
                break
        if not replaced:
            out.append(line)

    return text[:start] + "\n".join(out) + text[end:]

cam0 = {
    "qCM_x": "0.0",
    "qCM_y": "0.0",
    "qCM_z": "0.0",
    "qCM_w": "1.0",
    "MrMC_x": "-0.005520000122487545",
    "MrMC_y": "0.005100000184029341",
    "MrMC_z": "0.011739999987185001",
}

cam1 = {
    "qCM_x": "0.0",
    "qCM_y": "0.0",
    "qCM_z": "0.0",
    "qCM_w": "1.0",
    "MrMC_x": "0.04454804304987192",
    "MrMC_y": "0.005100000184029341",
    "MrMC_z": "0.011739999987185001",
}

text = patch_block(text, "Camera0", cam0)
text = patch_block(text, "Camera1", cam1)

text = text.replace("doFrameVisualisation true;", "doFrameVisualisation false;")

text = text.replace("}Camera1", "}\nCamera1")
text = text.replace("}Init", "}\nInit")
text = text.replace("}ImgUpdate", "}\nImgUpdate")
text = text.replace("}PoseUpdate", "}\nPoseUpdate")

p.write_text(text)

print("===== CHECK FIRST 40 LINES =====")
for i, line in enumerate(p.read_text().splitlines(), 1):
    if 1 <= i <= 40:
        print(f"{i}:{line}")
PY
```

Camera0 đúng phải giống:

```text
Camera0
{
    qCM_x 0.0;
    qCM_y 0.0;
    qCM_z 0.0;
    qCM_w 1.0;

    MrMC_x -0.005520000122487545;
    MrMC_y 0.005100000184029341;
    MrMC_z 0.011739999987185001;
}
```

Camera1 đúng phải giống:

```text
Camera1
{
    qCM_x 0.0;
    qCM_y 0.0;
    qCM_z 0.0;
    qCM_w 1.0;

    MrMC_x 0.04454804304987192;
    MrMC_y 0.005100000184029341;
    MrMC_z 0.011739999987185001;
}
```

Lưu ý format rất quan trọng. Không được để:

```text
}Camera1
```

Phải là:

```text
}
Camera1
```

---

## 11. Vì sao cần QoS relay

Source ROVIO hiện tại tạo subscription trước khi đọc param topic:

```cpp
subImu_  = this->create_subscription<sensor_msgs::msg::Imu>("imu0", 1000, ...)
subImg0_ = this->create_subscription<sensor_msgs::msg::Image>("cam0/image_raw", 100, ...)
subImg1_ = this->create_subscription<sensor_msgs::msg::Image>("cam1/image_raw", 100, ...)
```

Sau đó mới đọc:

```cpp
imu_topic = readAndDeclareParam<std::string>("imu_topic", imu_topic);
cam0_topic = readAndDeclareParam<std::string>("cam0_topic", cam0_topic);
cam1_topic = readAndDeclareParam<std::string>("cam1_topic", cam1_topic);
```

Vì vậy param `imu_topic`, `cam0_topic`, `cam1_topic` trong launch gần như không có tác dụng với subscriber thật.

ROVIO thực tế cần các topic:

```text
/imu0
/cam0/image_raw
/cam1/image_raw
```

Trong khi D435i publish:

```text
/camera/camera/imu
/camera/camera/infra1/image_rect_raw
/camera/camera/infra2/image_rect_raw
```

Nên cần relay:

```text
/camera/camera/imu                       -> /imu0
/camera/camera/infra1/image_rect_raw     -> /cam0/image_raw
/camera/camera/infra2/image_rect_raw     -> /cam1/image_raw
```

---

## 12. Tạo package `rovio_qos_relay`

```bash
cd ~/rovio_ws/src

mkdir -p rovio_qos_relay/rovio_qos_relay
touch rovio_qos_relay/rovio_qos_relay/__init__.py
mkdir -p rovio_qos_relay/resource
touch rovio_qos_relay/resource/rovio_qos_relay
```

Tạo `package.xml`:

```bash
cat > rovio_qos_relay/package.xml <<'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>rovio_qos_relay</name>
  <version>0.0.1</version>
  <description>QoS relay for RealSense D435i to ROVIO</description>
  <maintainer email="hann@example.com">hann</maintainer>
  <license>MIT</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>

  <buildtool_depend>ament_python</buildtool_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF
```

Tạo `setup.py`:

```bash
cat > rovio_qos_relay/setup.py <<'EOF'
from setuptools import setup

package_name = 'rovio_qos_relay'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hann',
    maintainer_email='hann@example.com',
    description='QoS relay for RealSense D435i to ROVIO',
    license='MIT',
    entry_points={
        'console_scripts': [
            'd435i_to_rovio_relay = rovio_qos_relay.d435i_to_rovio_relay:main',
        ],
    },
)
EOF
```

Tạo `setup.cfg`:

```bash
cat > rovio_qos_relay/setup.cfg <<'EOF'
[develop]
script_dir=$base/lib/rovio_qos_relay
[install]
install_scripts=$base/lib/rovio_qos_relay
EOF
```

Tạo relay node:

```bash
cat > rovio_qos_relay/rovio_qos_relay/d435i_to_rovio_relay.py <<'EOF'
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)
from sensor_msgs.msg import Image, Imu


class D435iToRovioRelay(Node):
    def __init__(self):
        super().__init__('d435i_to_rovio_relay')

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        reliable_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.pub_imu = self.create_publisher(Imu, '/imu0', reliable_qos)
        self.pub_cam0 = self.create_publisher(Image, '/cam0/image_raw', reliable_qos)
        self.pub_cam1 = self.create_publisher(Image, '/cam1/image_raw', reliable_qos)

        self.sub_imu = self.create_subscription(
            Imu,
            '/camera/camera/imu',
            self.pub_imu.publish,
            sensor_qos,
        )
        self.sub_cam0 = self.create_subscription(
            Image,
            '/camera/camera/infra1/image_rect_raw',
            self.pub_cam0.publish,
            sensor_qos,
        )
        self.sub_cam1 = self.create_subscription(
            Image,
            '/camera/camera/infra2/image_rect_raw',
            self.pub_cam1.publish,
            sensor_qos,
        )

        self.get_logger().info('Relaying D435i -> ROVIO:')
        self.get_logger().info('/camera/camera/imu -> /imu0')
        self.get_logger().info('/camera/camera/infra1/image_rect_raw -> /cam0/image_raw')
        self.get_logger().info('/camera/camera/infra2/image_rect_raw -> /cam1/image_raw')


def main(args=None):
    rclpy.init(args=args)
    node = D435iToRovioRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

Build relay:

```bash
cd ~/rovio_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rovio_qos_relay --symlink-install
```

Kiểm tra:

```bash
source install/setup.bash
ros2 pkg executables rovio_qos_relay
```

Kỳ vọng:

```text
rovio_qos_relay d435i_to_rovio_relay
```

---

## 13. Test relay riêng

Chạy RealSense:

```bash
source /opt/ros/humble/setup.bash
source ~/rovio_ws/install/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  enable_color:=false \
  enable_depth:=false \
  enable_infra1:=true \
  enable_infra2:=true \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=2
```

Terminal khác chạy relay:

```bash
source /opt/ros/humble/setup.bash
source ~/rovio_ws/install/setup.bash

ros2 run rovio_qos_relay d435i_to_rovio_relay
```

Terminal khác kiểm tra:

```bash
source /opt/ros/humble/setup.bash
source ~/rovio_ws/install/setup.bash

ros2 topic list | grep -E "^/imu0$|^/cam0/image_raw$|^/cam1/image_raw$"

ros2 topic hz /cam0/image_raw
ros2 topic hz /cam1/image_raw
ros2 topic hz /imu0
```

Kết quả mong muốn:

```text
/cam0/image_raw ~30 Hz
/cam1/image_raw ~30 Hz
/imu0           ~200 Hz
```

---

## 14. Chạy full pipeline thủ công

Terminal 1: RealSense

```bash
source /opt/ros/humble/setup.bash
source ~/rovio_ws/install/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  enable_color:=false \
  enable_depth:=false \
  enable_infra1:=true \
  enable_infra2:=true \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=2
```

Terminal 2: relay

```bash
source /opt/ros/humble/setup.bash
source ~/rovio_ws/install/setup.bash

ros2 run rovio_qos_relay d435i_to_rovio_relay
```

Terminal 3: ROVIO

```bash
source /opt/ros/humble/setup.bash
source ~/rovio_ws/install/setup.bash

ros2 run rovio rovio_node \
  --ros-args \
  -p filter_config:=/home/hann/rovio_ws/src/rovio/cfg/rovio_d435i.info \
  -p camera0_config:=/home/hann/rovio_ws/src/rovio/cfg/d435i_infra1.yaml \
  -p camera1_config:=/home/hann/rovio_ws/src/rovio/cfg/d435i_infra2.yaml \
  -p use_sim_time:=false \
  -p map_frame:=map \
  -p world_frame:=world \
  -p camera_frame:=camera \
  -p imu_frame:=imu
```

Terminal 4: kiểm tra output

```bash
source /opt/ros/humble/setup.bash
source ~/rovio_ws/install/setup.bash

ros2 node list
ros2 topic list | grep rovio
ros2 topic hz /rovio/odometry
ros2 topic echo --once /rovio/odometry
ros2 topic echo --once /rovio/health
```

---

## 15. Tạo script chạy full pipeline

Tạo file:

```bash
cd ~/rovio_ws

cat > run_d435i_rovio_headless.sh <<'EOF'
#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source ~/rovio_ws/install/setup.bash

echo "===== START REALSENSE D435I ====="
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=false \
  enable_depth:=false \
  enable_infra1:=true \
  enable_infra2:=true \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=2 \
  > /tmp/rs_for_rovio.log 2>&1 &

RS_LAUNCH_PID=$!

sleep 8

echo "===== START D435I -> ROVIO QOS RELAY ====="
ros2 run rovio_qos_relay d435i_to_rovio_relay \
  > /tmp/rovio_qos_relay_full.log 2>&1 &

RELAY_PID=$!

sleep 3

echo "===== START ROVIO NODE ====="
ros2 run rovio rovio_node \
  --ros-args \
  -p filter_config:=/home/hann/rovio_ws/src/rovio/cfg/rovio_d435i.info \
  -p camera0_config:=/home/hann/rovio_ws/src/rovio/cfg/d435i_infra1.yaml \
  -p camera1_config:=/home/hann/rovio_ws/src/rovio/cfg/d435i_infra2.yaml \
  -p use_sim_time:=false \
  -p map_frame:=map \
  -p world_frame:=world \
  -p camera_frame:=camera \
  -p imu_frame:=imu \
  > /tmp/rovio_node_d435i.log 2>&1 &

ROVIO_PID=$!

cleanup() {
  echo
  echo "===== CLEANUP D435I ROVIO PIPELINE ====="

  kill "$ROVIO_PID" 2>/dev/null || true
  kill "$RELAY_PID" 2>/dev/null || true
  kill "$RS_LAUNCH_PID" 2>/dev/null || true

  RS_PIDS=$(ps -eo pid,cmd | awk '/[r]ealsense2_camera_node/ {print $1}')
  if [ -n "$RS_PIDS" ]; then
    echo "Killing RealSense node PID(s): $RS_PIDS"
    kill $RS_PIDS 2>/dev/null || true
  fi

  RELAY_PIDS=$(ps -eo pid,cmd | awk '/[d]435i_to_rovio_relay/ {print $1}')
  if [ -n "$RELAY_PIDS" ]; then
    echo "Killing relay PID(s): $RELAY_PIDS"
    kill $RELAY_PIDS 2>/dev/null || true
  fi

  ROVIO_PIDS=$(ps -eo pid,cmd | awk '/[r]ovio_node/ {print $1}')
  if [ -n "$ROVIO_PIDS" ]; then
    echo "Killing ROVIO PID(s): $ROVIO_PIDS"
    kill $ROVIO_PIDS 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo
echo "===== PIPELINE IS RUNNING ====="
echo "Check odom from another SSH terminal:"
echo "  source /opt/ros/humble/setup.bash && source ~/rovio_ws/install/setup.bash && ros2 topic hz /rovio/odometry"
echo
echo "Press Ctrl+C to stop."

wait
EOF

chmod +x run_d435i_rovio_headless.sh
```

Chạy script:

```bash
cd ~/rovio_ws
./run_d435i_rovio_headless.sh
```

Dừng script:

```text
Ctrl + C
```

---

## 16. Lệnh xem odometry

Xem odometry liên tục:

```bash
source /opt/ros/humble/setup.bash
source ~/rovio_ws/install/setup.bash

ros2 topic echo /rovio/odometry
```

Xem 1 message rồi dừng:

```bash
source /opt/ros/humble/setup.bash
source ~/rovio_ws/install/setup.bash

ros2 topic echo --once /rovio/odometry
```

Xem tần số:

```bash
source /opt/ros/humble/setup.bash
source ~/rovio_ws/install/setup.bash

ros2 topic hz /rovio/odometry
```

Xem health:

```bash
source /opt/ros/humble/setup.bash
source ~/rovio_ws/install/setup.bash

ros2 topic echo --once /rovio/health
```

---

## 17. Cách tắt nếu lỡ chạy nền

Kiểm tra process:

```bash
ps -eo pid,ppid,stat,cmd | grep -E '[r]ovio_node|[d]435i_to_rovio_relay|[r]ealsense2_camera_node|[r]s_launch'
```

Ví dụ thấy:

```text
22884 ros2 launch realsense2_camera ...
22898 realsense2_camera_node ...
22969 ros2 run rovio_qos_relay ...
22971 d435i_to_rovio_relay
23014 ros2 run rovio rovio_node ...
23015 rovio_node ...
```

Tắt theo PID cụ thể:

```bash
kill 23015 23014 22971 22969 22898 22884 2>/dev/null || true
```

Kiểm tra lại:

```bash
ps -eo pid,ppid,stat,cmd | grep -E '[r]ovio_node|[d]435i_to_rovio_relay|[r]ealsense2_camera_node|[r]s_launch' || echo "ALL STOPPED"
```

---

## 18. Kết quả test đã đạt

### 18.1 Full pipeline test

ROVIO publish được:

```text
/rovio/odometry
/rovio/pose_with_covariance_stamped
/rovio/health
/rovio/pcl
/rovio/markers
/rovio/patch
/rovio/transform
/rovio/imu_biases
```

Tần số odometry:

```text
/rovio/odometry ~29-30 Hz
```

---

### 18.2 Static drift test 60 giây

Start:

```text
x = -0.0006278
y = -0.0007567
z = -0.0006726
```

End sau 60 giây:

```text
x = -0.0015569
y = -0.0040371
z = -0.0074905
```

Drift:

```text
dx ≈ -0.00093 m
dy ≈ -0.00328 m
dz ≈ -0.00682 m

norm ≈ 0.0076 m / 60s
```

Health:

```text
valid_feature_ratio:   0.92
tracked_feature_ratio: 0.92
speed_deviation:       0.0013
accel_deviation:       9.816
```

Kết luận:

```text
Static test: PASS
```

---

### 18.3 Motion sanity test

Start:

```text
x =  0.12625
y = -0.22947
z = -0.12010
```

End:

```text
x =  0.04650
y = -0.20178
z = -0.07203
```

Delta:

```text
dx ≈ -0.0798 m
dy ≈  0.0277 m
dz ≈  0.0481 m

norm ≈ 0.097 m
```

Health:

```text
valid_feature_ratio:   0.80
tracked_feature_ratio: 0.80
speed_deviation:       0.00163
```

Kết luận:

```text
Motion sanity: PASS basic
Metric scale: chưa xác nhận tuyệt đối vì chưa có ground truth.
```

---

## 19. Lỗi divergence và cách nhận biết

Nếu `/rovio/odometry` hiện kiểu:

```text
x = 592
y = -469
z = -95
```

hoặc tăng liên tục lên hàng trăm/hàng nghìn mét, nghĩa là:

```text
ROVIO filter divergence / filter nổ
```

Nguyên nhân đã gặp:

```text
qCM sai do hiểu nhầm frame IMU.
```

Sai:

```text
qCM = [-0.5, 0.5, -0.5, 0.5]
```

Đúng cho setup hiện tại:

```text
qCM = [0, 0, 0, 1]
```

Vì:

```text
/camera/camera/imu frame_id = camera_imu_optical_frame
/camera/camera/infra1/camera_info frame_id = camera_infra1_optical_frame
```

Tức là IMU data đã ở optical frame rồi, không cần xoay thêm.

Nếu đang chạy mà pose nổ, nên tắt sạch process rồi chạy lại:

```bash
ps -eo pid,ppid,stat,cmd | grep -E '[r]ovio_node|[d]435i_to_rovio_relay|[r]ealsense2_camera_node|[r]s_launch'
```

Kill đúng PID rồi restart.

---

## 20. Known issues

### 20.1 Timestamp của `/rovio/odometry`

Hiện `/rovio/odometry` có stamp kiểu:

```text
sec: 1
nanosec: 779953...
```

Đây không phải wall-clock time.

Nếu sau này đưa sang PX4/MAVROS/external vision, cần xử lý lại timestamp.

---

### 20.2 RealSense warning

Có lúc RealSense báo:

```text
Hardware Notification: Left MIPI error
```

Pipeline vẫn chạy, nhưng nếu odometry bị giật/drop frame, cần kiểm tra:

```text
USB cable
USB port
nguồn cấp
camera profile
RealSense firmware
```

---

### 20.3 Launch mặc định của ROVIO không phù hợp headless

Không nên dùng trực tiếp:

```text
ros2_rovio_node_launch.py
```

vì nó:

```text
- dùng topic mặc định không đúng D435i
- có image_view, không phù hợp headless
- set use_sim_time true
- dùng Euroc config thay vì D435i config
```

---

## 21. Baseline hiện tại

```text
Build ROVIO: PASS
D435i infra1/infra2/imu: PASS
QoS relay: PASS
ROVIO odometry publish: PASS
Static drift 60s: PASS
Motion sanity: PASS basic
```

Pipeline hiện tại đủ để làm baseline ROVIO headless.

Bước tiếp theo nên làm:

```text
1. Fix timestamp cho /rovio/odometry.
2. Chuẩn hóa frame nếu đưa sang PX4/MAVROS.
3. Test motion có ground truth để kiểm tra metric scale.
4. Xem xét sửa source ROVIO để đọc topic params trước khi create_subscription, khi đó có thể bỏ relay topic-name hardcode.
```

---

## 22. Lệnh chạy nhanh hằng ngày

Chạy pipeline:

```bash
cd ~/rovio_ws
./run_d435i_rovio_headless.sh # (x-right,y-front, z-up)
```
```bash
cd ~/rovio_ws
./run_d435i_rovio_headless_flu.sh # đổi hướng (x-front, y-left, z-up)
```
Terminal khác xem odometry:

```bash
source /opt/ros/humble/setup.bash
source ~/rovio_ws/install/setup.bash
ros2 topic echo /rovio/odometry
```
```bash
source /opt/ros/humble/setup.bash
source ~/rovio_ws/install/setup.bash
#xem tất cả 
ros2 topic echo /rovio/odometry_flu 
# Chỉ xem position
ros2 topic echo /rovio/odometry_flu --field pose.pose.position
```
Xem health:

```bash
source /opt/ros/humble/setup.bash
source ~/rovio_ws/install/setup.bash
ros2 topic echo --once /rovio/health
```

Xem tần số:

```bash
source /opt/ros/humble/setup.bash
source ~/rovio_ws/install/setup.bash
ros2 topic hz /rovio/odometry
```

Dừng pipeline:

```text
Ctrl + C
```
