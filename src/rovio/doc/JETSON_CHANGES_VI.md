# Tóm tắt các thay đổi cho Jetson Orin

## Tổng quan
Đã cập nhật ROVIO để hỗ trợ đầy đủ cho NVIDIA Jetson Orin và các nền tảng ARM64 khác. 

## Các file đã thay đổi

### 1. CMakeLists.txt
**Thay đổi chính**: Tự động phát hiện kiến trúc và áp dụng cờ tối ưu hóa phù hợp

**Trước đây** (chỉ cho x86_64):
```cmake
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
```

**Bây giờ** (hỗ trợ cả x86_64 và ARM64):
```cmake
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  # Tối ưu cho ARM64 (Jetson Orin, Xavier)
  message(STATUS "Detected ARM64 architecture - using ARM-specific optimizations")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mcpu=native -O3")
  # Thêm tối ưu NEON nếu có
  check_cxx_compiler_flag("-mfpu=neon" HAS_NEON)
  if(HAS_NEON)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon")
  endif()
else()
  # Tối ưu cho x86_64
  message(STATUS "Detected x86_64 architecture - using x86-specific optimizations")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
endif()
```

**Lợi ích**:
- Tự động phát hiện kiến trúc CPU
- Sử dụng cờ tối ưu phù hợp cho từng nền tảng
- Hỗ trợ NEON SIMD trên ARM64
- Không cần sửa file thủ công khi build trên nền tảng khác

### 2. README.md
**Thêm các phần mới**:

#### a) Bảng nền tảng được hỗ trợ
| Platform | Architecture | OS | ROS 2 | Status |
|----------|--------------|----|---------|----|
| Desktop/Laptop | x86_64 | Ubuntu 22.04 | Humble | ✅ Tested |
| NVIDIA Jetson Orin | ARM64 | Ubuntu 20.04/22.04 | Humble | ✅ Supported |
| NVIDIA Jetson Xavier | ARM64 | Ubuntu 18.04/20.04 | Humble | ⚠️ Experimental |

#### b) Hướng dẫn cài đặt cho Jetson
- Yêu cầu JetPack 5.0+
- Hướng dẫn tạo swap space (cho RAM < 16GB)
- Cấu hình power mode (MAXN)
- Build với parallel jobs giới hạn (tránh hết RAM)

#### c) Quick Start cho Jetson
```bash
cd ~/rovio_ws && build_rovio_jetson  # Build tối ưu cho Jetson
```

#### d) Tối ưu hiệu suất
Giảm tải tính toán bằng cách giảm số lượng features:
```cmake
set(ROVIO_NMAXFEATURE 15)  # Giảm từ 25
set(ROVIO_NLEVELS 3)       # Giảm từ 4
set(ROVIO_PATCHSIZE 4)     # Giảm từ 6
```

### 3. scripts/rovio_commands.sh
**Thêm function mới**:
```bash
function build_rovio_jetson() {
  echo "Building ROVIO for Jetson Orin/Xavier (ARM64)..."
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 4
}
```

**Giải thích**:
- Giới hạn 4 jobs song song (thay vì unlimited)
- Tránh hết RAM khi build trên Jetson
- Vẫn đủ nhanh (tận dụng 4 cores)

### 4. cfg/d435i_jetson_config.yaml (File mới)
**File cấu hình tối ưu cho Jetson**:
- Độ phân giải giảm: 640x480 (thay vì 1280x720)
- Camera intrinsics được scale theo tỷ lệ 0.5
- MaxFeatureCount: 15 (giảm từ 25)
- DetectorFrequency: 3 Hz (giảm từ 5 Hz)
- PatchSize: 6 (giảm từ 8)
- SearchAreaSize: 30 (giảm từ 40)

**Hiệu suất mong đợi trên Jetson Orin Nano**:
- Tốc độ xử lý: 20-30 Hz
- CPU usage: 40-50% (2 cores)
- Memory: 2-3 GB

### 5. doc/JetsonSetup.md (File mới)
**Hướng dẫn chi tiết**:
- Yêu cầu phần cứng/phần mềm
- Cài đặt từng bước
- Tối ưu hiệu suất
- Troubleshooting
- Benchmark performance

## Cách sử dụng

### Trên x86_64 (Desktop/Laptop)
```bash
cd ~/rovio_ws
source src/rovio/scripts/rovio_commands.sh
build_rovio  # Build bình thường
```

### Trên ARM64 (Jetson Orin/Xavier)
```bash
cd ~/rovio_ws
source src/rovio/scripts/rovio_commands.sh

# Tạo swap nếu RAM < 16GB
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Set power mode MAXN
sudo nvpmodel -m 0
sudo jetson_clocks

# Build cho Jetson
build_rovio_jetson
```

### Chạy ROVIO với config tối ưu cho Jetson
```bash
cd ~/rovio_ws
source install/setup.bash
ros2 launch rovio ros2_d435i_rovio_launch.py \
  config_file:=$(ros2 pkg prefix rovio)/share/rovio/cfg/d435i_jetson_config.yaml
```

## Lợi ích

### 1. Tự động hóa
- Build system tự động phát hiện kiến trúc
- Không cần thay đổi code thủ công
- Một codebase cho tất cả nền tảng

### 2. Hiệu suất
- Tối ưu compiler flags cho từng kiến trúc
- Config riêng cho Jetson giảm tải CPU/RAM
- Swap space và power mode được hướng dẫn

### 3. Khả năng mở rộng
- Dễ dàng thêm hỗ trợ kiến trúc mới
- Config files rõ ràng, dễ điều chỉnh
- Documentation đầy đủ

## Lưu ý quan trọng

### Khi build lần đầu trên Jetson
1. **Đảm bảo có đủ RAM/SWAP**: Build có thể cần > 8GB
2. **Kiểm tra nhiệt độ**: Jetson có thể nóng khi build
   ```bash
   watch -n 1 cat /sys/devices/virtual/thermal/thermal_zone*/temp
   ```
3. **Sử dụng build_rovio_jetson**: Giới hạn parallel jobs tránh crash

### Khi chạy ROVIO trên Jetson
1. **Monitoring resources**:
   ```bash
   sudo tegrastats  # Check CPU/GPU/RAM/Temp
   ```
2. **Nếu chậm (<15 Hz)**:
   - Giảm `ROVIO_NMAXFEATURE` xuống 10-12
   - Giảm độ phân giải camera
   - Kiểm tra power mode: `sudo nvpmodel -q`

### Camera với Jetson
- **RealSense D435i**: Yêu cầu librealsense2 được build cho ARM64
- **USB 3.0**: Sử dụng cổng USB 3.0 (màu xanh) để bandwidth đủ
- **Độ phân giải**: 640x480 @ 30 FPS được khuyến nghị

## Kiểm tra thay đổi

### Xác minh build system phát hiện đúng
```bash
cd ~/rovio_ws
rm -rf build install log
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Trong output, phải thấy:
# "Detected ARM64 architecture - using ARM-specific optimizations"
# (trên Jetson)
# hoặc
# "Detected x86_64 architecture - using x86-specific optimizations"  
# (trên Desktop)
```

### Kiểm tra performance
```bash
# Terminal 1: Chạy ROVIO
ros2 launch rovio ros2_d435i_rovio_launch.py

# Terminal 2: Monitor resources
sudo tegrastats

# Terminal 3: Check rate
ros2 topic hz /rovio/odometry
# Phải thấy: average rate: 20-30 Hz (trên Jetson Orin Nano)
```

## Tài liệu tham khảo
- [README.md](../README.md) - Main documentation
- [JetsonSetup.md](JetsonSetup.md) - Chi tiết về Jetson
- [CustomSetup.md](CustomSetup.md) - Cấu hình camera-IMU tùy chỉnh

## Hỗ trợ
Nếu gặp vấn đề khi build hoặc chạy trên Jetson:
1. Kiểm tra [JetsonSetup.md - Troubleshooting](JetsonSetup.md#troubleshooting)
2. Mở issue trên GitHub với:
   - Jetson model (Orin Nano/NX/AGX)
   - JetPack version: `sudo apt-cache show nvidia-jetpack`
   - Build error/output đầy đủ
   - Resource usage: `sudo tegrastats` output
