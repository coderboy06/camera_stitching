import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class StitchNode(Node):
    def __init__(self):
        super().__init__('stitch_node')
        self.get_logger().info('Stitching Camera Views with Car Model Overlay')

        # ROS2 subscribers for four camera feeds
        self.left_subscription = self.create_subscription(
            Image, '/left_camera/image_raw', self.left_callback, 10
        )
        self.right_subscription = self.create_subscription(
            Image, '/right_camera/image_raw', self.right_callback, 10
        )
        self.front_subscription = self.create_subscription(
            Image, '/front_camera/image_raw', self.front_callback, 10
        )
        self.back_subscription = self.create_subscription(
            Image, '/back_camera/image_raw', self.back_callback, 10
        )

        # Publisher for stitched image
        self.stitched_publisher = self.create_publisher(Image, 'stitched_camera/image_raw', 10)

        self.bridge = CvBridge()
        self.left_frame = None
        self.right_frame = None
        self.front_frame = None
        self.back_frame = None

        # Load a simple car model image (replace with your own model if available)
        self.car_model = np.zeros((600, 800, 3), dtype=np.uint8)
        cv2.putText(self.car_model, 'CAR MODEL', (300, 300), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.rectangle(self.car_model, (350, 250), (450, 350), (0, 255, 0), 2)  # Simple car-like visualization

    def left_callback(self, msg):
        self.left_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.get_logger().info('Received left frame')
        self.try_stitch()

    def right_callback(self, msg):
        self.right_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.get_logger().info('Received right frame')
        self.try_stitch()

    def front_callback(self, msg):
        self.front_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.get_logger().info('Received front frame')
        self.try_stitch()

    def back_callback(self, msg):
        self.back_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.get_logger().info('Received back frame')
        self.try_stitch()

    def try_stitch(self):
        # Proceed if all frames are available
        if self.left_frame is not None and self.right_frame is not None and \
           self.front_frame is not None and self.back_frame is not None:
            self.get_logger().info('Stitching frames')

            # Resize frames to fit the car model view
            left_resized = cv2.resize(self.left_frame, (200, 300))
            right_resized = cv2.resize(self.right_frame, (200, 300))
            front_resized = cv2.resize(self.front_frame, (400, 200))
            back_resized = cv2.resize(self.back_frame, (400, 200))

            # Create a blank canvas for the stitched output
            stitched_frame = np.copy(self.car_model)

            # Place the camera views around the car model
            stitched_frame[50:600, 0:200] = left_resized  # Left camera view
            stitched_frame[50:600, 600:800] = right_resized  # Right camera view
            stitched_frame[0:200, 200:600] = front_resized  # Front camera view
            stitched_frame[400:600, 200:600] = back_resized  # Back camera view

            # Publish the stitched frame
            stitched_msg = self.bridge.cv2_to_imgmsg(stitched_frame, encoding='bgr8')
            self.stitched_publisher.publish(stitched_msg)
            self.get_logger().info('Published stitched frame with car model overlay')


def main(args=None):
    rclpy.init(args=args)
    node = StitchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down stitch node')
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
