import rclpy, cv2, numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

class Viewer(Node):
    def __init__(self):
        super().__init__('viewer')
        self.create_subscription(Image, '/camera/image_annotated', self.cb, qos_profile_sensor_data)
        cv2.namedWindow('AeroScout camera', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('AeroScout camera', 640, 480)
    def cb(self, m):
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        cv2.imshow('AeroScout camera', img)
        cv2.waitKey(1)

rclpy.init()
rclpy.spin(Viewer())
