# README — Publish ROVIO Odometry to PX4 through MAVROS

## 1. Mục tiêu

Mục tiêu của pipeline này là truyền odometry từ ROVIO sang PX4 để PX4 nhận được dữ liệu tại:

```sh
listener vehicle_visual_odometry
```

Pipeline tổng thể:

```text
D435i
→ RealSense ROS2
→ relay D435i to ROVIO
→ ROVIO
→ /rovio/odometry
→ rovio_odom_to_flu.py
→ /rovio/odometry_flu
→ rovio_to_mavros_odometry.py
→ /mavros/odometry/out
→ MAVROS
→ TELEM2
→ PX4
→ vehicle_visual_odometry
```

Hiện tại mục tiêu là **publish đúng position/orientation vào PX4**, chưa bật EKF fusion vội.

---

## 2. Frame convention cần nhớ

### 2.1 D435i optical frame

D435i optical frame gốc:

```text
X = right
Y = down
Z = forward
```

### 2.2 ROVIO raw output quan sát được

Sau khi qua ROVIO, raw `/rovio/odometry` quan sát thực tế theo hướng:

```text
X = right
Y = forward
Z = up
```

Frame này gần giống ROS ENU nếu ta coi:

```text
X = East / right
Y = North / forward
Z = up
```

### 2.3 ROS body frame FLU

ROS `base_link` thường dùng:

```text
X = forward
Y = left
Z = up
```

Gọi là FLU.

### 2.4 PX4 body/world frame

PX4 nhận `vehicle_visual_odometry` theo logic:

```text
X = forward
Y = right
Z = down
```

Vì vậy khi kiểm tra trong PX4:

```text
Đi tới trước  → position X tăng
Đi sang phải → position Y tăng
Nhấc lên     → position Z giảm
Hạ xuống     → position Z tăng
```

---

## 3. Các file chính trong workspace

Workspace:

```sh
~/rovio_ws
```

Các file quan trọng:

```text
~/rovio_ws/run_d435i_rovio_headless.sh
~/rovio_ws/run_d435i_rovio_headless_flu.sh

~/rovio_ws/src/rovio_qos_relay/rovio_qos_relay/d435i_to_rovio_relay.py
~/rovio_ws/src/rovio_qos_relay/rovio_qos_relay/rovio_odom_to_flu.py
~/rovio_ws/src/rovio_qos_relay/rovio_qos_relay/rovio_to_mavros_odometry.py
```

Ý nghĩa:

```text
d435i_to_rovio_relay.py
  Relay topic D435i sang topic ROVIO cần:
  /camera/camera/infra1/image_rect_raw → /cam0/image_raw
  /camera/camera/infra2/image_rect_raw → /cam1/image_raw
  /camera/camera/imu                  → /imu0

rovio_odom_to_flu.py
  Chuyển /rovio/odometry sang /rovio/odometry_flu.
  Topic này dùng để debug dễ hiểu theo kiểu:
  X forward, Y left, Z up.

rovio_to_mavros_odometry.py
  Chuyển /rovio/odometry_flu sang /mavros/odometry/out.
  Đây là topic MAVROS dùng để gửi visual odometry sang PX4.
```

---

## 4. Những chỉnh sửa đã làm để PX4 nhận đúng

### 4.1 Tạo `/rovio/odometry_flu`

Trong `rovio_odom_to_flu.py`, mapping position từ ROVIO raw sang FLU:

```text
X_flu =  Y_rovio
Y_flu = -X_rovio
Z_flu =  Z_rovio
```

Ý nghĩa:

```text
ROVIO raw:
X = right
Y = forward
Z = up

FLU:
X = forward
Y = left
Z = up
```

Sau mapping:

```text
Đi tới trước → /rovio/odometry_flu position.x tăng
Đi sang trái → /rovio/odometry_flu position.y tăng
Nhấc lên     → /rovio/odometry_flu position.z tăng
```

### 4.2 Sửa orientation trong `rovio_odom_to_flu.py`

Orientation không được đổi bằng cách swap `qx qy qz qw`.

Phải đổi bằng quaternion multiplication.

Cấu trúc đã dùng:

```text
q_cam_in_flu_world = q_axis * q_old * q_axis_inv
q_base_in_flu_world = q_cam_in_flu_world * q_cam_to_base
q_final = q_base_in_flu_world * q_body_roll_fix
```

Ý nghĩa:

```text
q_axis
  Đổi world/body axis từ raw ROVIO sang FLU.

q_cam_to_base
  Đổi child frame từ camera/ROVIO frame sang body/base_link của quad.

q_body_roll_fix
  Correction thực nghiệm -90 độ quanh X.
  Lý do: trước đó PX4 thấy visual roll khoảng +90 độ khi quad đang cân bằng.
```

Sau correction, khi quad nằm cân bằng, PX4 thấy:

```text
vehicle_visual_odometry:
Roll  ≈ 0 deg
Pitch ≈ -2 đến -3 deg
Yaw   ≈ gần 0 deg hoặc offset nhỏ
```

### 4.3 Chuyển `/rovio/odometry_flu` sang MAVROS ENU

MAVROS `/mavros/odometry/out` không nên nhận trực tiếp `X forward, Y left, Z up`.

MAVROS mong dữ liệu ROS ENU:

```text
X = right/east
Y = forward/north
Z = up
```

Do đó trong `rovio_to_mavros_odometry.py`, position được đổi:

```text
X_enu = -Y_flu
Y_enu =  X_flu
Z_enu =  Z_flu
```

Nếu thay công thức FLU ở trên vào, net effect position là:

```text
X_enu = X_rovio
Y_enu = Y_rovio
Z_enu = Z_rovio
```

Điều này hợp lý vì raw ROVIO đang quan sát được là:

```text
X = right
Y = forward
Z = up
```

### 4.4 Sửa orientation từ FLU world sang MAVROS ENU

Trong `rovio_to_mavros_odometry.py`, orientation cũng phải đổi world frame, không chỉ position.

Công thức:

```text
q_enu = Q_FLU_WORLD_TO_ENU * q_flu
```

Trong đó:

```text
Q_FLU_WORLD_TO_ENU = +90 deg quanh Z
```

Lưu ý:

```text
Đổi world/parent frame:
  nhân bên trái

Đổi body/child frame:
  nhân bên phải
```

---

## 5. Cài MAVROS

Nếu máy chưa có MAVROS:

```sh
sudo apt update
sudo apt install -y ros-humble-mavros ros-humble-mavros-extras
```

Cài GeographicLib dataset:

```sh
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

Kiểm tra:

```sh
source /opt/ros/humble/setup.bash

ros2 pkg list | grep -E "^mavros$|^mavros_msgs$|^mavros_extras$"
ros2 pkg executables mavros
```

Kỳ vọng có:

```text
mavros
mavros_msgs
mavros_extras
mavros mavros_node
```

---

## 6. Kết nối PX4 với Jetson

PX4 nối Jetson qua TELEM2 bằng USB-UART CP2102.

Kết nối dây:

```text
PX4 TELEM2 TX → USB-UART RX
PX4 TELEM2 RX → USB-UART TX
PX4 GND       → USB-UART GND
```

Không cần nối 5V nếu Jetson đã có nguồn riêng.

Kiểm tra serial device:

```sh
echo "===== SERIAL DEVICES ====="
ls -lah /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true

echo
echo "===== SERIAL BY ID ====="
ls -lah /dev/serial/by-id/ 2>/dev/null || true
```

Kỳ vọng:

```text
/dev/ttyUSB0
/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

---

## 7. Chạy pipeline

Cần 3 terminal chính.

---

### Terminal 1 — Bật MAVROS

```sh
source /opt/ros/humble/setup.bash

ros2 launch mavros px4.launch \
  fcu_url:=serial:///dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0:921600
```

Giữ terminal này chạy.

Kiểm tra ở terminal khác:

```sh
source /opt/ros/humble/setup.bash

ros2 topic echo --once /mavros/state
timeout 5 ros2 topic hz /mavros/imu/data || true
```

Kỳ vọng:

```text
connected: true
/mavros/imu/data ≈ 50 Hz
```

---

### Terminal 2 — Bật D435i + ROVIO + FLU converter

```sh
cd ~/rovio_ws
source /opt/ros/humble/setup.bash
source ~/rovio_ws/install/setup.bash

./run_d435i_rovio_headless_flu.sh
```

File này chạy:

```text
RealSense D435i
→ d435i_to_rovio_relay
→ rovio_node
→ rovio_odom_to_flu
```

Output chính:

```text
/rovio/odometry
/rovio/odometry_flu
/rovio/health
```

---

### Terminal 3 — Bridge ROVIO sang MAVROS

```sh
source /opt/ros/humble/setup.bash
source ~/rovio_ws/install/setup.bash

ros2 run rovio_qos_relay rovio_to_mavros_odometry \
  --ros-args \
  -p input_odom_topic:=/rovio/odometry_flu \
  -p output_odom_topic:=/mavros/odometry/out \
  -p use_health_gate:=true \
  -p min_tracked_feature_ratio:=0.60 \
  -p max_speed_deviation:=0.50 \
  -p frame_id:=map \
  -p child_frame_id:=base_link
```

Node này làm:

```text
/rovio/odometry_flu
→ đổi sang MAVROS ENU
→ /mavros/odometry/out
→ MAVROS
→ PX4
```

---

## 8. Kiểm tra trên Jetson

Mở terminal kiểm tra:

```sh
source /opt/ros/humble/setup.bash
source ~/rovio_ws/install/setup.bash

echo "===== MAVROS STATE ====="
ros2 topic echo --once /mavros/state

echo
echo "===== ROVIO RAW HZ ====="
timeout 6 ros2 topic hz /rovio/odometry || true

echo
echo "===== ROVIO FLU HZ ====="
timeout 6 ros2 topic hz /rovio/odometry_flu || true

echo
echo "===== MAVROS ODOM OUT HZ ====="
timeout 6 ros2 topic hz /mavros/odometry/out || true

echo
echo "===== ROVIO HEALTH ====="
timeout 5 ros2 topic echo --once /rovio/health || true

echo
echo "===== MAVROS ODOM OUT SAMPLE ====="
timeout 5 ros2 topic echo --once /mavros/odometry/out | sed -n '1,80p'
```

Kỳ vọng:

```text
/mavros/state connected: true
/rovio/odometry      ≈ 25–30 Hz
/rovio/odometry_flu  ≈ 25–30 Hz
/mavros/odometry/out ≈ 25–30 Hz

tracked_feature_ratio > 0.6
speed_deviation thấp
```

---

## 9. Kiểm tra trên PX4 MAVLink Console

Trong QGroundControl → MAVLink Console:

```sh
listener vehicle_visual_odometry
```

Kỳ vọng có dữ liệu:

```text
TOPIC: vehicle_visual_odometry
position: [...]
q: [...]
velocity: [...]
pose_frame: 2
velocity_frame: 3
```

So sánh với attitude thật của FC:

```sh
listener vehicle_attitude
```

Khi quad nằm cân bằng:

```text
vehicle_attitude:
Roll/Pitch gần 0

vehicle_visual_odometry:
Roll/Pitch cũng gần 0
```

Yaw có thể chưa trùng tuyệt đối vì ROVIO có yaw origin riêng.

---

## 10. Test chiều position trong PX4

Trong PX4, dùng:

```sh
listener vehicle_visual_odometry
```

Test từng trục:

### 10.1 Test X forward

Đặt quad đứng yên, đọc position:

```text
position: [x0, y0, z0]
```

Đẩy quad về phía trước 20–30 cm, đọc lại:

```text
position: [x1, y1, z1]
```

Kỳ vọng:

```text
x1 > x0
```

Tức là:

```text
Đi tới trước → PX4 position X tăng
```

### 10.2 Test Y right

Đẩy quad sang phải 20–30 cm.

Kỳ vọng:

```text
Đi sang phải → PX4 position Y tăng
Đi sang trái → PX4 position Y giảm
```

### 10.3 Test Z down

Nhấc quad lên 20–30 cm.

Kỳ vọng:

```text
Nhấc lên → PX4 position Z giảm
Hạ xuống → PX4 position Z tăng
```

Lý do: PX4 dùng Z-down.

---

## 11. Test orientation

### 11.1 Cân bằng

Khi quad nằm cân bằng:

```sh
listener vehicle_attitude
listener vehicle_visual_odometry
```

Kỳ vọng:

```text
vehicle_attitude roll/pitch gần 0
vehicle_visual_odometry roll/pitch gần 0
```

### 11.2 Roll

Nghiêng quad sang phải/trái.

Kỳ vọng:

```text
vehicle_visual_odometry Roll đổi cùng chiều tương đối với vehicle_attitude Roll
```

### 11.3 Pitch

Chúi/ngẩng đầu quad.

Kỳ vọng:

```text
vehicle_visual_odometry Pitch đổi cùng chiều tương đối với vehicle_attitude Pitch
```

### 11.4 Yaw

Yaw chưa nên fuse vội.

Lý do:

```text
ROVIO yaw là yaw tương đối theo thời điểm start.
PX4 yaw thường theo estimator/mag/heading riêng.
Yaw có thể có offset.
```

---

## 12. Health gate

`rovio_to_mavros_odometry.py` đang có health gate:

```text
tracked_feature_ratio >= 0.60
speed_deviation <= 0.50
```

Nếu ROVIO tracking xấu:

```text
tracked_feature_ratio thấp
speed_deviation cao
ROVIO divergence
```

thì node bridge sẽ không publish tiếp sang MAVROS.

Đây là hành vi an toàn.

Nếu PX4 `vehicle_visual_odometry` bị đứng timestamp, kiểm tra:

```sh
source /opt/ros/humble/setup.bash
source ~/rovio_ws/install/setup.bash

echo "===== ROVIO FLU HZ ====="
timeout 6 ros2 topic hz /rovio/odometry_flu || true

echo
echo "===== MAVROS ODOM OUT HZ ====="
timeout 6 ros2 topic hz /mavros/odometry/out || true

echo
echo "===== ROVIO HEALTH ====="
timeout 5 ros2 topic echo --once /rovio/health || true
```

Cách đọc:

```text
/rovio/odometry_flu có Hz nhưng /mavros/odometry/out không có:
  bridge bị health gate chặn hoặc node bridge chết.

/rovio/odometry_flu không có Hz:
  ROVIO hoặc RealSense đang lỗi.

/mavros/odometry/out có Hz nhưng PX4 không update:
  kiểm tra MAVROS state, TELEM2, QGC console.
```

---

## 13. Debug RealSense nếu ROVIO không publish odometry

Nếu không có `/rovio/odometry`, kiểm tra từ đầu:

```sh
source /opt/ros/humble/setup.bash
source ~/rovio_ws/install/setup.bash

echo "===== PROCESS CHECK ====="
ps -eo pid,ppid,stat,cmd | grep -E '[r]ealsense2_camera_node|[r]s_launch|[d]435i_to_rovio_relay|[r]ovio_node|[r]ovio_odom_to_flu|[r]ovio_to_mavros_odometry|[m]avros' || echo "NO RELATED PROCESS"

echo
echo "===== NODE CHECK ====="
ros2 node list | sort | grep -E "camera|d435i|rovio|mavros" || echo "NO RELATED ROS NODE"

echo
echo "===== TOPIC CHECK ====="
ros2 topic list | sort | grep -E "camera/camera/infra|camera/camera/imu|cam0|cam1|imu0|rovio|mavros/odometry" || echo "NO RELATED TOPIC"
```

Nếu RealSense lỗi kiểu:

```text
xioctl(VIDIOC_QBUF) failed when requesting new frame
No such device
Frames didn't arrived within 5 seconds
IR stream start failure
```

thì làm:

```sh
ps -eo pid,ppid,stat,cmd | grep -E '[r]ealsense2_camera_node|[r]s_launch'

kill <PID_REALSense_NODE> 2>/dev/null || true
sleep 2

for d in /dev/video*; do
  echo "--- $d"
  fuser -v "$d" 2>/dev/null || true
done
```

Sau đó rút D435i, chờ 5 giây, cắm lại cổng USB 3.0 chắc chắn.

Kiểm tra:

```sh
lsusb | grep -i -E "intel|realsense|8086" || echo "NO REALSENSE USB"
ls -lah /dev/video* 2>/dev/null || echo "NO VIDEO DEVICES"
```

---

## 14. Test RealSense native trước khi chạy ROVIO

Không chạy ROVIO khi RealSense chưa pass.

Test RealSense only:

```sh
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

Terminal khác kiểm tra:

```sh
source /opt/ros/humble/setup.bash

timeout 6 ros2 topic hz /camera/camera/infra1/image_rect_raw || true
timeout 6 ros2 topic hz /camera/camera/infra2/image_rect_raw || true
timeout 6 ros2 topic hz /camera/camera/imu || true
```

Kỳ vọng:

```text
infra1 ≈ 30 Hz
infra2 ≈ 30 Hz
imu    ≈ 200 Hz
```

Nếu pass mới chạy lại:

```sh
cd ~/rovio_ws
./run_d435i_rovio_headless_flu.sh
```

---

## 15. Dừng pipeline

Dừng ROVIO pipeline:

```sh
ps -eo pid,ppid,stat,cmd | grep -E '[r]ovio_node|[r]ovio_odom_to_flu|[r]ovio_to_mavros_odometry|[d]435i_to_rovio_relay|[r]ealsense2_camera_node|[r]s_launch'
```

Kill đúng PID:

```sh
kill <PID1> <PID2> <PID3> 2>/dev/null || true
sleep 2
```

Kiểm tra lại:

```sh
ps -eo pid,ppid,stat,cmd | grep -E '[r]ovio_node|[r]ovio_odom_to_flu|[r]ovio_to_mavros_odometry|[d]435i_to_rovio_relay|[r]ealsense2_camera_node|[r]s_launch' || echo "NO ROVIO PIPELINE PROCESS"
```

Dừng MAVROS nếu cần:

```sh
ps -eo pid,ppid,stat,cmd | grep -E '[m]avros'
kill <MAVROS_PID> 2>/dev/null || true
```

---

## 16. Trạng thái hiện tại

Đã đạt:

```text
PX4 nhận được vehicle_visual_odometry
Position đúng logic PX4:
  Forward → X tăng
  Right   → Y tăng
  Up      → Z giảm

Orientation đã tốt hơn:
  Khi cân bằng roll/pitch gần 0
  Không còn lỗi roll lệch 90 deg như ban đầu
```

Chưa nên làm ngay:

```text
Chưa bật EKF2 fuse yaw/orientation
Chưa dùng ROVIO yaw làm yaw source chính
Chưa bay thật với external vision nếu chưa test đủ failsafe
```

Hướng an toàn tiếp theo:

```text
1. Test position-only thật chắc.
2. Test health gate khi ROVIO mất tracking.
3. Sau đó mới bật EKF2 external vision position fusion.
4. Yaw/orientation fusion làm sau.
```

---

## 17. Quick run checklist

### Terminal 1

```sh
source /opt/ros/humble/setup.bash

ros2 launch mavros px4.launch \
  fcu_url:=serial:///dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0:921600
```

### Terminal 2

```sh
cd ~/rovio_ws
source /opt/ros/humble/setup.bash
source ~/rovio_ws/install/setup.bash

./run_d435i_rovio_headless_flu.sh
```

### Terminal 3

```sh
source /opt/ros/humble/setup.bash
source ~/rovio_ws/install/setup.bash

ros2 run rovio_qos_relay rovio_to_mavros_odometry \
  --ros-args \
  -p input_odom_topic:=/rovio/odometry_flu \
  -p output_odom_topic:=/mavros/odometry/out \
  -p use_health_gate:=true \
  -p min_tracked_feature_ratio:=0.60 \
  -p max_speed_deviation:=0.50 \
  -p frame_id:=map \
  -p child_frame_id:=base_link
```

### QGC MAVLink Console

```sh
listener vehicle_visual_odometry
listener vehicle_attitude
```

Expected PX4 motion signs:

```text
Forward → X tăng
Right   → Y tăng
Up      → Z giảm
```
