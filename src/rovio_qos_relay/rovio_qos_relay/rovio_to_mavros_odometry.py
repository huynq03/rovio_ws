import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import math
from geometry_msgs.msg import Quaternion


def q_normalize(q):
    n = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if n <= 1e-12:
        return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    return Quaternion(x=q.x/n, y=q.y/n, z=q.z/n, w=q.w/n)


def q_mul(a, b):
    return Quaternion(
        x=a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        y=a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        z=a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
        w=a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    )


# Convert our ROVIO FLU world:
#   X forward, Y left, Z up
# to MAVROS expected ENU:
#   X east/right, Y north/forward, Z up
#
# Mapping:
#   X_enu = -Y_flu
#   Y_enu =  X_flu
#   Z_enu =  Z_flu
#
# This is +90 deg around Z.
Q_FLU_WORLD_TO_ENU = Quaternion(
    x=0.0,
    y=0.0,
    z=0.70710678118,
    w=0.70710678118,
)


def flu_world_pos_to_enu(p):
    return (-p.y, p.x, p.z)


def flu_world_quat_to_enu(q):
    return q_normalize(q_mul(Q_FLU_WORLD_TO_ENU, q_normalize(q)))

try:
    from rovio_interfaces.msg import Health
    HAS_ROVIO_HEALTH = True
except Exception:
    HAS_ROVIO_HEALTH = False


class RovioToMavrosOdometry(Node):
    def __init__(self):
        super().__init__('rovio_to_mavros_odometry')

        self.declare_parameter('input_odom_topic', '/rovio/odometry_flu')
        self.declare_parameter('output_odom_topic', '/mavros/odometry/out')
        self.declare_parameter('health_topic', '/rovio/health')
        self.declare_parameter('use_health_gate', True)
        self.declare_parameter('min_tracked_feature_ratio', 0.60)
        self.declare_parameter('max_speed_deviation', 0.50)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')

        self.input_odom_topic = self.get_parameter('input_odom_topic').value
        self.output_odom_topic = self.get_parameter('output_odom_topic').value
        self.health_topic = self.get_parameter('health_topic').value
        self.use_health_gate = bool(self.get_parameter('use_health_gate').value)
        self.min_tracked_feature_ratio = float(self.get_parameter('min_tracked_feature_ratio').value)
        self.max_speed_deviation = float(self.get_parameter('max_speed_deviation').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        self.last_health = None
        self.sent_count = 0
        self.drop_count = 0

        self.pub = self.create_publisher(
            Odometry,
            self.output_odom_topic,
            30,
        )

        self.sub_odom = self.create_subscription(
            Odometry,
            self.input_odom_topic,
            self.odom_cb,
            30,
        )

        if HAS_ROVIO_HEALTH:
            self.sub_health = self.create_subscription(
                Health,
                self.health_topic,
                self.health_cb,
                10,
            )
        else:
            self.sub_health = None
            self.get_logger().warn(
                'rovio_interfaces.msg.Health not available. Health gate will be disabled.'
            )
            self.use_health_gate = False

        self.get_logger().info(f'Input odom:  {self.input_odom_topic}')
        self.get_logger().info(f'Output odom: {self.output_odom_topic}')
        self.get_logger().info(f'Health gate: {self.use_health_gate}')
        self.get_logger().info(f'frame_id={self.frame_id}, child_frame_id={self.child_frame_id}')

    def health_cb(self, msg):
        self.last_health = msg

    def health_ok(self):
        if not self.use_health_gate:
            return True

        if self.last_health is None:
            return False

        tracked = float(self.last_health.tracked_feature_ratio)
        speed_dev = float(self.last_health.speed_deviation)

        if tracked < self.min_tracked_feature_ratio:
            return False

        if speed_dev > self.max_speed_deviation:
            return False

        return True

    def odom_cb(self, msg):
        if not self.health_ok():
            self.drop_count += 1
            if self.drop_count % 30 == 0:
                if self.last_health is None:
                    self.get_logger().warn('Dropping odom: no ROVIO health yet')
                else:
                    self.get_logger().warn(
                        'Dropping odom: tracked_feature_ratio='
                        f'{self.last_health.tracked_feature_ratio:.3f}, '
                        f'speed_deviation={self.last_health.speed_deviation:.3f}'
                    )
            return

        out = Odometry()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.child_frame_id = self.child_frame_id

        # Convert pose from our FLU-world convention into MAVROS ENU convention.
        out.pose = msg.pose
        px, py, pz = flu_world_pos_to_enu(msg.pose.pose.position)
        out.pose.pose.position.x = px
        out.pose.pose.position.y = py
        out.pose.pose.position.z = pz
        out.pose.pose.orientation = flu_world_quat_to_enu(msg.pose.pose.orientation)

        # Keep twist as-is for now. Twist is expressed in child_frame_id/base_link.
        # MAVROS will handle body FLU -> FRD conversion.
        out.twist = msg.twist

        self.pub.publish(out)

        self.sent_count += 1
        if self.sent_count % 30 == 0:
            p = out.pose.pose.position
            self.get_logger().info(
                f'Published MAVROS odom #{self.sent_count}: '
                f'x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = RovioToMavrosOdometry()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
