import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CamNodeRight(Node):
    def __init__(self):
        super().__init__('cam_node_front')
        self.publisher_ = self.create_publisher(Image, '/camera_front', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)  # Right camera (usually index 1)

        if not self.cap.isOpened():
            self.get_logger().error('Unable to open the right camera')
        else:
            self.get_logger().info('Right Camera Node is Running')
            self.timer = self.create_timer(0.1, self.publish_frame)  # Publish at 10 Hz

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing frame from right camera')
        else:
            self.get_logger().error('Failed to capture frame')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CamNodeRight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
