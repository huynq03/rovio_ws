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
