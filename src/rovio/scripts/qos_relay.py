#!/usr/bin/env python3
"""
QoS Relay Node: Subscribe to RealSense topics with Best Effort QoS
and republish with Reliable QoS for ROVIO compatibility.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, Imu


class QoSRelayNode(Node):
    def __init__(self):
        super().__init__('qos_relay')
        
        # QoS profile for subscribing to RealSense (Best Effort)
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # QoS profile for publishing to ROVIO (Reliable)
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Image relay: /camera/camera/infra1/image_rect_raw -> /cam0/image_raw
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/infra1/image_rect_raw',
            self.image_callback,
            qos_best_effort
        )
        self.image_pub = self.create_publisher(
            Image,
            '/cam0/image_raw',
            qos_reliable
        )
        
        # IMU relay: /camera/camera/imu -> /imu0
        self.imu_sub = self.create_subscription(
            Imu,
            '/camera/camera/imu',
            self.imu_callback,
            qos_best_effort
        )
        self.imu_pub = self.create_publisher(
            Imu,
            '/imu0',
            qos_reliable
        )
        
        self.get_logger().info('QoS Relay Node started')
        self.get_logger().info('  Image: /camera/camera/infra1/image_rect_raw -> /cam0/image_raw')
        self.get_logger().info('  IMU:   /camera/camera/imu -> /imu0')
        
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
