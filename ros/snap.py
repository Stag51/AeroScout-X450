import rclpy, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

OUT = '/mnt/c/Users/devsa/annotated.png'

class Snap(Node):
    def __init__(self):
        super().__init__('snap')
        self.bridge = CvBridge()
        self.done = False
        self.create_subscription(Image, '/camera/image_annotated', self.cb, 10)

    def cb(self, msg):
        cv2.imwrite(OUT, self.bridge.imgmsg_to_cv2(msg, 'bgr8'))
        self.get_logger().info('saved ' + OUT)
        self.done = True

rclpy.init()
n = Snap()
while rclpy.ok() and not n.done:
    rclpy.spin_once(n, timeout_sec=0.5)
n.destroy_node()
rclpy.shutdown()
