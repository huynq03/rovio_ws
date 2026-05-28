import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


def q_mul(a, b):
    return Quaternion(
        x=a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        y=a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        z=a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
        w=a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    )


class RovioOdomToFlu(Node):
    def __init__(self):
        super().__init__('rovio_odom_to_flu')

        self.sub = self.create_subscription(
            Odometry,
            '/rovio/odometry',
            self.cb,
            50,
        )

        self.pub = self.create_publisher(
            Odometry,
            '/rovio/odometry_flu',
            50,
        )

        self.get_logger().info(
            'Publishing /rovio/odometry_flu with mapping: '
            'X_new=Y_old, Y_new=-X_old, Z_new=Z_old'
        )

    def cb(self, msg):
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = 'base_link_flu'

        # New frame: X forward, Y left, Z up
        # Assumption: ROVIO old Y = forward, old X = right, old Z = up.
        out.pose.pose.position.x = msg.pose.pose.position.y
        out.pose.pose.position.y = -msg.pose.pose.position.x
        out.pose.pose.position.z = msg.pose.pose.position.z

        # For first test, keep orientation unchanged.
        # Position-axis correctness is tested first.
        out.pose.pose.orientation = msg.pose.pose.orientation

        out.twist.twist.linear.x = msg.twist.twist.linear.y
        out.twist.twist.linear.y = -msg.twist.twist.linear.x
        out.twist.twist.linear.z = msg.twist.twist.linear.z

        out.twist.twist.angular.x = msg.twist.twist.angular.y
        out.twist.twist.angular.y = -msg.twist.twist.angular.x
        out.twist.twist.angular.z = msg.twist.twist.angular.z

        out.pose.covariance = msg.pose.covariance
        out.twist.covariance = msg.twist.covariance

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = RovioOdomToFlu()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
