import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from cv_bridge import CvBridge


def quaternion_to_matrix(x, y, z, w):
    """Quaternion -> 3x3 rotation matrix."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class LaneGeometryNode(Node):
    """Reprojects the white-boundary mask from color_segmentation_node onto
    the ground plane (inverse perspective mapping) using the camera's known
    intrinsics/extrinsics, fits the left/right road-edge lines in the car's
    own body frame, and derives the car's position within the lane, its
    heading relative to the road, and the road's curvature ahead.

    Fully independent of color_segmentation_node - this script only knows
    about a binary mask + camera geometry, never about color thresholds,
    the same separation of concerns as car_sim's road.py/render.py split.
    Every quantity here is in the car's own body frame (x-forward, y-left,
    z-up); no world/global localization is assumed, matching what a real
    onboard perception stack would actually have access to.
    """

    def __init__(self):
        super().__init__('lane_geometry_node')
        self.declare_parameter('max_range_m', 25.0)   # ignore reprojected points beyond this
        self.declare_parameter('min_points_per_side', 15)  # confidence gate

        self.max_range_m = self.get_parameter('max_range_m').value
        self.min_points_per_side = self.get_parameter('min_points_per_side').value

        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None
        self.cam_pos = None       # (3,) camera position in car body frame
        self.R_body_from_cam = None  # (3,3)

        self.output_pub = self.create_publisher(String, '/detection/output', 10)
        self.create_subscription(CameraInfo, '/car/camera/camera_info', self._on_camera_info, 10)
        self.create_subscription(PoseStamped, '/car/camera/extrinsics', self._on_extrinsics, 10)
        self.create_subscription(Image, '/detection/segmentation_image', self._on_mask, 10)

    def _on_camera_info(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def _on_extrinsics(self, msg):
        p = msg.pose.position
        o = msg.pose.orientation
        self.cam_pos = np.array([p.x, p.y, p.z])
        self.R_body_from_cam = quaternion_to_matrix(o.x, o.y, o.z, o.w)

    def _on_mask(self, msg):
        if self.fx is None or self.cam_pos is None:
            return  # camera_info/extrinsics not received yet

        mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        ys, xs = np.nonzero(mask)
        if len(xs) == 0:
            return

        # Pixel -> camera-frame ray direction (unnormalized; only the
        # direction matters for a ray-plane intersection).
        d_cam = np.stack([
            (xs - self.cx) / self.fx,
            (ys - self.cy) / self.fy,
            np.ones_like(xs, dtype=np.float64),
        ], axis=1)
        d_body = d_cam @ self.R_body_from_cam.T

        # Ray-plane intersection with the ground (body-frame z=0). Only
        # rays pointing meaningfully downward hit the ground in front of
        # the car; near-horizontal rays (near the horizon) give huge,
        # unreliable distances, which the max_range cutoff below rejects.
        dz = d_body[:, 2]
        pointing_down = dz < -1e-6
        t = np.where(pointing_down, -self.cam_pos[2] / np.where(pointing_down, dz, -1.0), -1.0)
        X = t * d_body[:, 0]
        Y = t * d_body[:, 1]

        valid = pointing_down & (t > 0.0) & (np.hypot(X, Y) <= self.max_range_m)
        X, Y = X[valid], Y[valid]

        left = Y > 0.0
        right = Y < 0.0
        Xl, Yl = X[left], Y[left]
        Xr, Yr = X[right], Y[right]
        if len(Xl) < self.min_points_per_side or len(Xr) < self.min_points_per_side:
            return  # not enough confident points on one or both sides - skip this tick

        try:
            a_l, b_l, c_l = np.polyfit(Xl, Yl, 2)
            a_r, b_r, c_r = np.polyfit(Xr, Yr, 2)
        except np.linalg.LinAlgError:
            return

        # Everything evaluated at X=0 (the car's own position).
        denom = c_l - c_r
        if abs(denom) < 1e-6:
            return
        position = max(-1.0, min(1.0, (c_l + c_r) / denom))

        b_avg = (b_l + b_r) / 2.0
        heading = math.atan(b_avg)  # road tangent angle vs. car forward axis; +ve = road bends left

        a_avg = (a_l + a_r) / 2.0
        curvature = 2.0 * a_avg / (1.0 + b_avg ** 2) ** 1.5  # +ve = curving left (see docstring)
        eps_curv = 1e-6
        curvature_clamped = math.copysign(max(abs(curvature), eps_curv), curvature) \
            if curvature != 0.0 else eps_curv
        curvature_radius = -1.0 / curvature_clamped  # flip sign: output is +ve = right, -ve = left

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        payload = {
            'stamp': stamp,
            'position': position,
            'heading': heading,
            'curvature_radius': curvature_radius,
        }
        out = String()
        out.data = json.dumps(payload)
        self.output_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LaneGeometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
