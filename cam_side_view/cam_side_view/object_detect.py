import rclpy
from rclpy.node import Node
import cv2
import numpy as np

class ObjectDetectNode(Node):
    def __init__(self):
        super().__init__('object_detect')
        self.declare_parameter('weights_path', 'yolov4.weights')
        self.declare_parameter('config_path', 'yolov4.cfg')
        self.declare_parameter('labels_path', 'coco.names')

        weights_path = self.get_parameter('weights_path').get_parameter_value().string_value
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        labels_path = self.get_parameter('labels_path').get_parameter_value().string_value

        self.labels = None
        try:
            with open(labels_path, 'r') as f:
                self.labels = f.read().strip().split('\n')
        except FileNotFoundError:
            self.get_logger().warn(f"Labels file '{labels_path}' not found. Using default labels.")
            self.labels = ["Class1", "Class2", "Class3"]  # Fallback labels

        self.net = cv2.dnn.readNet(weights_path, config_path)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

        self.subscriber = self.create_subscription(
            np.ndarray, 'left_camera/image_raw', self.left_camera_callback, 10)

    def left_camera_callback(self, frame):
        frame = self.detect_objects(frame)
        cv2.imshow("Detected Objects", frame)
        cv2.waitKey(1)

    def detect_objects(self, frame):
        blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)
        layer_names = self.net.getLayerNames()
        output_layers = [layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]
        outputs = self.net.forward(output_layers)
        return self.process_detections(frame, outputs)

    def process_detections(self, frame, outputs):
        height, width = frame.shape[:2]
        boxes = []
        confidences = []
        class_ids = []

        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        if len(boxes) > 0:
            indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
            for i in indices.flatten():
                x, y, w, h = boxes[i]
                label = self.labels[class_ids[i]] if self.labels else f"Class {class_ids[i]}"
                confidence = confidences[i]

                # Draw bounding boxes and labels on the frame
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, f"{label}: {confidence:.2f}", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return frame

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
