#!/usr/bin/env python3
"""AeroScout X450 - YOLOv8 person detector on the drone camera.

Subscribes : /camera/image_raw        (sensor_msgs/Image)
Publishes  : /camera/image_annotated  (sensor_msgs/Image, boxes drawn)
             /target/bbox             (std_msgs/Float32MultiArray)
                 data = [found, cx, cy, w, h, confidence]
                 cx, cy, w, h are fractions of the image (0..1), found is 1.0 or 0.0
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from ultralytics import YOLO


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('conf', 0.4)
        model_name = self.get_parameter('model').value
        self.conf = float(self.get_parameter('conf').value)

        self.bridge = CvBridge()
        self.model = YOLO(model_name)
        # warm-up so the first real frame is not slow
        self.model.predict(np.zeros((480, 640, 3), dtype=np.uint8), verbose=False)

        self.image_pub = self.create_publisher(Image, '/camera/image_annotated', 10)
        self.bbox_pub = self.create_publisher(Float32MultiArray, '/target/bbox', 10)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.get_logger().info('YOLO detector ready. Looking for people on /camera/image_raw')

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        result = self.model.predict(frame, conf=self.conf, classes=[0], verbose=False)[0]
        boxes = result.boxes

        out = Float32MultiArray()
        if len(boxes) > 0:
            xywhn = boxes.xywhn.cpu().numpy()
            confs = boxes.conf.cpu().numpy()
            i = int((xywhn[:, 2] * xywhn[:, 3]).argmax())   # biggest person in the picture
            cx, cy, w, h = xywhn[i]
            out.data = [1.0, float(cx), float(cy), float(w), float(h), float(confs[i])]
            self.get_logger().info(
                f'person: centre=({cx:.2f}, {cy:.2f}) size=({w:.2f} x {h:.2f}) conf={confs[i]:.2f}',
                throttle_duration_sec=1.0)
        else:
            out.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.get_logger().info('no person in view', throttle_duration_sec=2.0)
        self.bbox_pub.publish(out)

        annotated = result.plot()
        ann_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        ann_msg.header = msg.header
        self.image_pub.publish(ann_msg)


def main():
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
