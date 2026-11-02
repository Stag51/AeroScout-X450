#!/usr/bin/env python3
"""AeroScout X450 - ground-truth logger for the distance-estimation project.

Every 0.1 s it writes one CSV row with:
  * where Gazebo says the drone camera and the person's feet really are
  * the true range / ground distance / forward / lateral offset between them
  * where the person's feet SHOULD appear in the image (exp_u, exp_v, 0..1)
  * the latest YOLO box from /target/bbox and how old it is

Optional: distance_estimator_node.py adds est_* columns (PX4-based estimate).
Needs the pose bridge running:
  ros2 run ros_gz_bridge parameter_bridge \
     "/world/default/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"

Frames: Gazebo world, x forward / y left / z up. Drone body is FLU.
Pitch is positive when the nose points DOWN.
"""
import csv
import math
import os
import re
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from tf2_msgs.msg import TFMessage

CAM_OFFSET = (0.064, 0.0, 0.0225)        # camera_link in the drone body frame, metres
HFOV, W, H = 1.74, 1280, 960             # from PX4's mono_cam model
FX = (W / 2) / math.tan(HFOV / 2)        # focal length in pixels (fy is the same)
FY = FX
N_POSES = 9                              # entries seen in dynamic_pose/info: 0 drone, 1 person, then links

FIELDS = ['t', 'drone_x', 'drone_y', 'drone_z', 'roll_deg', 'pitch_deg', 'yaw_deg',
          'cam_x', 'cam_y', 'cam_z', 'person_x', 'person_y', 'person_z',
          'true_range', 'true_ground', 'true_fwd', 'true_lat', 'exp_u', 'exp_v',
          'found', 'cx', 'cy', 'w', 'h', 'conf', 'bbox_age',
          'est_valid', 'est_range', 'est_ground', 'est_fwd', 'est_lat', 'est_h', 'est_roll', 'est_pitch', 'est_age']


def rot(q):
    """Rotation matrix (body -> world) from quaternion (x, y, z, w)."""
    x, y, z, w = q
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def rpy(q):
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def truth(dp, dq, pp):
    """True geometry between the drone camera and the person's feet."""
    R = rot(dq)
    cam = [dp[i] + sum(R[i][j] * CAM_OFFSET[j] for j in range(3)) for i in range(3)]
    d = [pp[i] - cam[i] for i in range(3)]
    roll, pitch, yaw = rpy(dq)
    out = {
        'cam': cam, 'rpy': (roll, pitch, yaw),
        'range': math.sqrt(sum(c * c for c in d)),
        'ground': math.hypot(d[0], d[1]),
        'fwd': d[0] * math.cos(yaw) + d[1] * math.sin(yaw),
        'lat': -d[0] * math.sin(yaw) + d[1] * math.cos(yaw),   # left is positive
        'exp_u': float('nan'), 'exp_v': float('nan'),
    }
    vb = [sum(R[j][i] * d[j] for j in range(3)) for i in range(3)]   # world -> body
    if vb[0] > 0.05:                                   # feet are in front of the camera
        out['exp_u'] = (W / 2 - FX * vb[1] / vb[0]) / W
        out['exp_v'] = (H / 2 - FY * vb[2] / vb[0]) / H
    return out


def resolve_order(drone='aeroscout_x450_0', person='target_person'):
    """Ask Gazebo once for the entry names, so we never rely on a fixed order."""
    out = subprocess.run(['gz', 'topic', '-e', '-t', '/world/default/dynamic_pose/info', '-n', '1'],
                         capture_output=True, text=True, timeout=20).stdout
    names = re.findall(r'^\s*name:\s*"([^"]+)"', out, re.M)
    if drone not in names or person not in names:
        raise SystemExit(f'Could not find {drone} and {person} in Gazebo. Found: {names}')
    return names.index(drone), names.index(person), len(names)


class Logger(Node):
    def __init__(self):
        super().__init__('metrics_logger')
        self.declare_parameter('out_dir', os.path.expanduser('~/aero_logs'))
        self.declare_parameter('rate', 10.0)
        out_dir = self.get_parameter('out_dir').value
        os.makedirs(out_dir, exist_ok=True)
        self.path = os.path.join(out_dir, time.strftime('run_%Y%m%d_%H%M%S.csv'))
        self.file = open(self.path, 'w', newline='')
        self.csv = csv.writer(self.file)
        self.csv.writerow(FIELDS)
        self.file.flush()
        self.i_drone, self.i_person, self.n_poses = resolve_order()
        self.get_logger().info(f'Gazebo order: drone=entry {self.i_drone}, person=entry {self.i_person}, '
                               f'{self.n_poses} entries')

        self.drone = None       # (position, quaternion)
        self.person = None      # position
        self.bbox = None
        self.bbox_t = None
        self.est = None
        self.est_t = None
        self.t0 = time.monotonic()
        self.rows = 0
        self.warned = False

        self.create_subscription(TFMessage, '/world/default/dynamic_pose/info', self.on_pose, 10)
        self.create_subscription(Float32MultiArray, '/target/bbox', self.on_bbox, 10)
        self.create_subscription(Float32MultiArray, '/target/estimate', self.on_est, 10)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self.tick)
        self.get_logger().info(f'Logging to {self.path}')

    def on_pose(self, msg):
        ts = msg.transforms
        if len(ts) != self.n_poses and not self.warned:
            self.warned = True
            self.get_logger().warn(f'dynamic_pose/info has {len(ts)} entries, expected {self.n_poses}. '
                                   'Restart the logger.')
        if len(ts) <= max(self.i_drone, self.i_person):
            return
        d, p = ts[self.i_drone].transform, ts[self.i_person].transform
        self.drone = ((d.translation.x, d.translation.y, d.translation.z),
                      (d.rotation.x, d.rotation.y, d.rotation.z, d.rotation.w))
        self.person = (p.translation.x, p.translation.y, p.translation.z)

    def on_bbox(self, msg):
        self.bbox = list(msg.data)
        self.bbox_t = time.monotonic()

    def on_est(self, msg):
        self.est = list(msg.data)
        self.est_t = time.monotonic()

    def tick(self):
        if self.drone is None or self.person is None:
            return
        dp, dq = self.drone
        tr = truth(dp, dq, self.person)
        roll, pitch, yaw = tr['rpy']
        nan = float('nan')
        b = self.bbox if self.bbox and len(self.bbox) >= 6 else [nan] * 6
        age = time.monotonic() - self.bbox_t if self.bbox_t else nan
        e = self.est if self.est and len(self.est) >= 8 else [nan] * 8
        est_age = time.monotonic() - self.est_t if self.est_t else nan
        t = time.monotonic() - self.t0
        self.csv.writerow([f'{t:.3f}', *[f'{v:.4f}' for v in dp],
                           *[f'{math.degrees(a):.3f}' for a in (roll, pitch, yaw)],
                           *[f'{v:.4f}' for v in tr['cam']], *[f'{v:.4f}' for v in self.person],
                           f'{tr["range"]:.4f}', f'{tr["ground"]:.4f}', f'{tr["fwd"]:.4f}',
                           f'{tr["lat"]:.4f}', f'{tr["exp_u"]:.4f}', f'{tr["exp_v"]:.4f}',
                           *[f'{v:.4f}' for v in b], f'{age:.3f}',
                           *[f'{v:.4f}' for v in e], f'{est_age:.3f}'])
        self.file.flush()
        self.rows += 1
        if self.rows % 20 == 1:
            bottom = b[2] + b[4] / 2 if b[0] == 1.0 else nan
            self.get_logger().info(
                f'drone z={dp[2]:.2f} pitch={math.degrees(pitch):+.1f} deg | person x={self.person[0]:.1f} '
                f'y={self.person[1]:+.1f} z={self.person[2]:+.2f} | true range={tr["range"]:.2f} m | '
                f'feet v: expected={tr["exp_v"]:.3f} box bottom={bottom:.3f}')


def main():
    rclpy.init()
    node = Logger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.file.close()
    print(f'Saved {node.rows} rows to {node.path}')


if __name__ == '__main__':
    main()