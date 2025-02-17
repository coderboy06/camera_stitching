import cv2
import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import sensor_msgs.msg as pc2
import std_msgs.msg


class CameraLidarCalibration(Node):
    def __init__(self):
        super().__init__('camera_lidar_calibration')
        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/left_camera/image_raw', self.image_callback, 10)
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/ouster/points', self.lidar_callback, 10)

        # Publishers
        self.image_pub = self.create_publisher(Image, '/processed_image', 10)
        self.lidar_pub = self.create_publisher(PointCloud2, '/transformed_lidar', 10)

        # Camera Parameters (Adjust these based on your calibration)
        self.camera_matrix = np.eye(3)
        self.dist_coeffs = np.zeros((1, 5))

        self.image = None
        self.lidar_data = None
        self.calibrated = True  # Assume intrinsic parameters are known

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_frame()

    def lidar_callback(self, msg):
        self.lidar_data = np.array(list(pc2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)))
        self.lidar_to_camera_transform()

    def process_frame(self):
        if self.image is None:
            return

        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        # Feature detection (ORB)
        orb = cv2.ORB_create()
        keypoints, descriptors = orb.detectAndCompute(gray, None)
        self.image = cv2.drawKeypoints(self.image, keypoints, None, color=(0, 255, 0))

        # Publish processed image
        processed_msg = self.bridge.cv2_to_imgmsg(self.image, encoding="bgr8")
        self.image_pub.publish(processed_msg)

    def lidar_to_camera_transform(self):
        if self.lidar_data is None or len(self.lidar_data) == 0:
            return

        # Convert LiDAR Data to Open3D PointCloud
        lidar_cloud = o3d.geometry.PointCloud()
        lidar_cloud.points = o3d.utility.Vector3dVector(self.lidar_data)

        # Transformation Matrix (Replace with your actual calibration matrix)
        transform_matrix = np.eye(4)
        transformed_lidar = lidar_cloud.transform(transform_matrix)

        self.get_logger().info("Lidar to Camera transformation applied")

        # Convert transformed LiDAR data to ROS2 PointCloud2 and publish
        transformed_points = np.asarray(transformed_lidar.points)
        lidar_msg = pc2.create_cloud_xyz32(
            std_msgs.msg.Header(stamp=self.get_clock().now().to_msg(), frame_id="camera_frame"),
            transformed_points
        )
        self.lidar_pub.publish(lidar_msg)


def main(args=None):
    rclpy.init(args=args)
    calib_node = CameraLidarCalibration()
    rclpy.spin(calib_node)
    calib_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
