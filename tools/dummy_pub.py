#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
from std_msgs.msg import Header
import numpy as np
import time

class DummyPublisher(Node):
    def __init__(self):
        super().__init__('dummy_pub')
        self.imu_pub = self.create_publisher(Imu, '/imu0', 10)
        self.img_pub = self.create_publisher(Image, '/cam0/image_raw', 10)
        self.img_pub2 = self.create_publisher(Image, '/cam1/image_raw', 10)
        self.start_time = self.get_clock().now()
        self.declare_parameter('rate_imu', 200)
        self.declare_parameter('rate_img', 10)
        self.rate_imu = self.get_parameter('rate_imu').get_parameter_value().integer_value
        self.rate_img = self.get_parameter('rate_img').get_parameter_value().integer_value
        self.imu_timer = self.create_timer(1.0/self.rate_imu, self.pub_imu)
        self.img_timer = self.create_timer(1.0/self.rate_img, self.pub_img)
        self.duration = 12.0
        self.img_h = 240
        self.img_w = 320
        # prepare image buffer
        self.img_data = (np.zeros((self.img_h, self.img_w), dtype=np.uint8)).tobytes()
        self.get_logger().info('Dummy publisher started')

    def pub_imu(self):
        now = self.get_clock().now()
        if (now - self.start_time).nanoseconds / 1e9 > self.duration:
            rclpy.shutdown()
            return
        m = Imu()
        m.header.stamp = now.to_msg()
        m.header.frame_id = 'imu_link'
        m.linear_acceleration.x = 0.0
        m.linear_acceleration.y = 0.0
        m.linear_acceleration.z = 9.81
        m.angular_velocity.x = 0.0
        m.angular_velocity.y = 0.0
        m.angular_velocity.z = 0.0
        self.imu_pub.publish(m)

    def pub_img(self):
        now = self.get_clock().now()
        if (now - self.start_time).nanoseconds / 1e9 > self.duration:
            rclpy.shutdown()
            return
        img = Image()
        img.header.stamp = now.to_msg()
        img.header.frame_id = 'cam0'
        img.height = self.img_h
        img.width = self.img_w
        img.encoding = 'mono8'
        img.is_bigendian = 0
        img.step = self.img_w
        img.data = list(self.img_data)
        self.img_pub.publish(img)
        # publish same image to cam1 as well (if single camera config expects 1 camera, OK)
        img2 = Image()
        img2.header = img.header
        img2.height = img.height
        img2.width = img.width
        img2.encoding = img.encoding
        img2.is_bigendian = img.is_bigendian
        img2.step = img.step
        img2.data = img.data
        self.img_pub2.publish(img2)

def main(args=None):
    rclpy.init(args=args)
    node = DummyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
