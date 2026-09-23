import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ColorSegmentationNode(Node):
    """HSV color-thresholds the front camera feed for the white road-boundary
    lines only (the yellow centerline is deliberately ignored). Publishes a
    binary mask that is both the human-viewable debug image and the exact
    input lane_geometry_node consumes - one topic serving both roles.

    Fully independent of lane_geometry_node: this script only knows about
    pixels and colors, never about camera geometry or world coordinates,
    the same separation of concerns as car_sim's road.py/render.py split.
    """

    def __init__(self):
        super().__init__('color_segmentation_node')
        self.declare_parameter('white_s_max', 60)     # low saturation = white/gray
        self.declare_parameter('white_v_min', 180)     # high value = bright

        self.white_s_max = self.get_parameter('white_s_max').value
        self.white_v_min = self.get_parameter('white_v_min').value

        self.bridge = CvBridge()
        self.mask_pub = self.create_publisher(Image, '/detection/segmentation_image', 10)
        self.create_subscription(Image, '/car/camera/image_raw', self._on_image, 10)

    def _on_image(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower = np.array([0, 0, self.white_v_min], dtype=np.uint8)
        upper = np.array([179, self.white_s_max, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        out = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        out.header = msg.header
        self.mask_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ColorSegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
