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

        # RealSense image/IMU topics are commonly best_effort.
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # ROVIO uses default rclcpp subscriptions, so publish reliable/default-style.
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
