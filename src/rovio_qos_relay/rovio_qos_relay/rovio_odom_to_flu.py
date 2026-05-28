import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


# Mapping:
#   X_new =  Y_old
#   Y_new = -X_old
#   Z_new =  Z_old
#
# Matrix A:
#   [ 0  1  0 ]
#   [-1  0  0 ]
#   [ 0  0  1 ]
#
# This is a -90 deg rotation around Z.
A = [
    [0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
]


def q_normalize(q):
    n = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if n <= 1e-12:
        return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    return Quaternion(x=q.x/n, y=q.y/n, z=q.z/n, w=q.w/n)


def q_conj(q):
    return Quaternion(x=-q.x, y=-q.y, z=-q.z, w=q.w)


def q_mul(a, b):
    return Quaternion(
        x=a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        y=a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        z=a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
        w=a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    )


def map_vec(v):
    # new = A * old
    return (
        v.y,
        -v.x,
        v.z,
    )


def rotate_cov_6x6(cov):
    # cov_new = T * cov_old * T^T
    # T = diag(A, A), for [x,y,z,roll,pitch,yaw]
    T = [[0.0 for _ in range(6)] for _ in range(6)]

    for r in range(3):
        for c in range(3):
            T[r][c] = A[r][c]
            T[r+3][c+3] = A[r][c]

    old = [[cov[r*6 + c] for c in range(6)] for r in range(6)]

    tmp = [[0.0 for _ in range(6)] for _ in range(6)]
    out = [[0.0 for _ in range(6)] for _ in range(6)]

    for r in range(6):
        for c in range(6):
            tmp[r][c] = sum(T[r][k] * old[k][c] for k in range(6))

    for r in range(6):
        for c in range(6):
            out[r][c] = sum(tmp[r][k] * T[c][k] for k in range(6))

    return [out[r][c] for r in range(6) for c in range(6)]


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

        # q_axis = rotation around Z by -90 deg.
        half = -math.pi / 4.0
        self.q_axis = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(half),
            w=math.cos(half),
        )
        self.q_axis_inv = q_conj(self.q_axis)

        # Camera is mounted facing forward with the quad.
        # D435i optical frame: X right, Y down, Z forward.
        # Quad base_link FLU: X forward, Y left, Z up.
        # R_world_base = R_world_camera * R_camera_base
        self.q_cam_to_base = Quaternion(
            x=0.5,
            y=-0.5,
            z=0.5,
            w=0.5,
        )

        # Extra correction observed from PX4:
        # vehicle_visual_odometry roll was about +90 deg while FC attitude roll was near 0.
        # Apply -90 deg around body X.
        self.q_body_roll_fix = Quaternion(
            x=-0.70710678118,
            y=0.0,
            z=0.0,
            w=0.70710678118,
        )

        self.get_logger().info(
            'Publishing /rovio/odometry_flu with full axis conversion: '
            'X_new=Y_old, Y_new=-X_old, Z_new=Z_old, orientation converted too.'
        )

    def cb(self, msg):
        out = Odometry()

        out.header = msg.header
        out.header.frame_id = 'world_flu'
        out.child_frame_id = 'base_link_flu'

        px, py, pz = map_vec(msg.pose.pose.position)
        out.pose.pose.position.x = px
        out.pose.pose.position.y = py
        out.pose.pose.position.z = pz

        q_old = q_normalize(msg.pose.pose.orientation)

        # First convert ROVIO world axes to FLU world axes.
        q_cam_in_flu_world = q_mul(q_mul(self.q_axis, q_old), self.q_axis_inv)

        # Then convert child frame from camera optical to quad base_link FLU.
        q_base_in_flu_world = q_mul(q_cam_in_flu_world, self.q_cam_to_base)

        # Final measured mounting correction.
        q_base_fixed_in_flu_world = q_mul(q_base_in_flu_world, self.q_body_roll_fix)

        out.pose.pose.orientation = q_normalize(q_base_fixed_in_flu_world)

        vx, vy, vz = map_vec(msg.twist.twist.linear)
        out.twist.twist.linear.x = vx
        out.twist.twist.linear.y = vy
        out.twist.twist.linear.z = vz

        wx, wy, wz = map_vec(msg.twist.twist.angular)
        out.twist.twist.angular.x = wx
        out.twist.twist.angular.y = wy
        out.twist.twist.angular.z = wz

        out.pose.covariance = rotate_cov_6x6(msg.pose.covariance)
        out.twist.covariance = rotate_cov_6x6(msg.twist.covariance)

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
