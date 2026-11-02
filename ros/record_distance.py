#!/usr/bin/env python3
"""Records the drone camera view with the YOLO box, the estimated distance and Gazebo's true distance.

Needs running: detector (4_detector.sh) and distance_estimator_node.py.
For the 'true distance' line also run the pose bridge (same one the logger uses). Without it the video
simply shows the estimate.
Output: ~/aero_videos/distance_<time>.mp4   (stop with Ctrl+C)
"""
import math
import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from tf2_msgs.msg import TFMessage

from metrics_logger import truth, resolve_order

FPS = 15
OUT_W, OUT_H = 960, 720
BLUE = (214, 120, 42)        # BGR of #2a78d6
WHITE = (255, 255, 255)
GREY = (200, 200, 200)
FONT = cv2.FONT_HERSHEY_SIMPLEX


def draw_overlay(frame, est, true_range, bbox):
    """frame: BGR image (OUT_W x OUT_H). est: /target/estimate list or None. bbox: /target/bbox list or None."""
    h, w = frame.shape[:2]
    # dot where the feet are: this pixel is what the distance is computed from
    if bbox is not None and len(bbox) >= 6 and bbox[0] > 0.5:
        fx, fy = int(bbox[1] * w), int((bbox[2] + bbox[4] / 2) * h)
        cv2.circle(frame, (fx, fy), 9, WHITE, 2, cv2.LINE_AA)
        cv2.circle(frame, (fx, fy), 5, BLUE, -1, cv2.LINE_AA)
    # translucent panel, top left
    panel = frame.copy()
    cv2.rectangle(panel, (0, 0), (430, 150 if true_range is not None else 100), (20, 20, 20), -1)
    cv2.addWeighted(panel, 0.6, frame, 0.4, 0, frame)
    cv2.rectangle(frame, (0, 0), (8, 150 if true_range is not None else 100), BLUE, -1)
    valid = est is not None and len(est) >= 8 and est[0] > 0.5
    cv2.putText(frame, 'ESTIMATED DISTANCE', (24, 32), FONT, 0.6, GREY, 1, cv2.LINE_AA)
    cv2.putText(frame, f'{est[1]:.2f} m' if valid else '-- m', (24, 78), FONT, 1.5, WHITE, 3, cv2.LINE_AA)
    if true_range is not None:
        cv2.putText(frame, f'Gazebo truth {true_range:.2f} m', (24, 112), FONT, 0.7, GREY, 1, cv2.LINE_AA)
        if valid:
            cv2.putText(frame, f'error {est[1] - true_range:+.2f} m', (24, 140), FONT, 0.7, GREY, 1, cv2.LINE_AA)
    return frame


class Recorder(Node):
    def __init__(self):
        super().__init__('record_distance')
        self.bridge = CvBridge()
        self.frame = self.bbox = self.est = None
        self.est_t = 0.0
        self.drone = self.person = None
        try:
            self.i_drone, self.i_person, _ = resolve_order()
        except SystemExit as e:
            self.get_logger().warn(f'No true distance in the video ({e}). Is the pose bridge running?')
            self.i_drone = self.i_person = None
        self.create_subscription(Image, '/camera/image_annotated', self.on_image, 5)
        self.create_subscription(Float32MultiArray, '/target/bbox', lambda m: setattr(self, 'bbox', list(m.data)), 10)
        self.create_subscription(Float32MultiArray, '/target/estimate', self.on_est, 10)
        if self.i_drone is not None:
            self.create_subscription(TFMessage, '/world/default/dynamic_pose/info', self.on_pose, 10)
        os.makedirs(os.path.expanduser('~/aero_videos'), exist_ok=True)
        self.path = os.path.expanduser(time.strftime('~/aero_videos/distance_%Y%m%d_%H%M%S.mp4'))
        self.writer = cv2.VideoWriter(self.path, cv2.VideoWriter_fourcc(*'mp4v'), FPS, (OUT_W, OUT_H))
        self.frames = 0
        self.create_timer(1.0 / FPS, self.tick)
        self.get_logger().info(f'Recording to {self.path}. Press Ctrl+C to stop.')

    def on_image(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.frame = cv2.resize(img, (OUT_W, OUT_H), interpolation=cv2.INTER_AREA)

    def on_est(self, msg):
        self.est = list(msg.data)
        self.est_t = time.monotonic()

    def on_pose(self, msg):
        ts = msg.transforms
        if len(ts) <= max(self.i_drone, self.i_person):
            return
        d, p = ts[self.i_drone].transform, ts[self.i_person].transform
        self.drone = ((d.translation.x, d.translation.y, d.translation.z),
                      (d.rotation.x, d.rotation.y, d.rotation.z, d.rotation.w))
        self.person = (p.translation.x, p.translation.y, p.translation.z)

    def tick(self):
        if self.frame is None:
            return
        est = self.est if time.monotonic() - self.est_t < 0.5 else None
        true_range = None
        if self.drone is not None and self.person is not None:
            true_range = truth(self.drone[0], self.drone[1], self.person)['range']
        self.writer.write(draw_overlay(self.frame.copy(), est, true_range, self.bbox))
        self.frames += 1

    def close(self):
        self.writer.release()
        print(f'Saved {self.frames} frames ({self.frames / FPS:.0f} s) to {self.path}')


def main():
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.close()


if __name__ == '__main__':
    main()