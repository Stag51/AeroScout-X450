#!/usr/bin/env python3
"""AeroScout X450 - live distance estimate from the person's box (ground-plane geometry).

Subscribes: /target/bbox, /fmu/out/vehicle_attitude, /fmu/out/vehicle_local_position_v1
Publishes:  /target/estimate = [valid, range, ground, fwd, lat, cam_height, roll_deg, pitch_down_deg]
            range = camera to feet, ground = horizontal distance, fwd/lat = in the drone's heading
            frame (lat positive to the left). All metres. valid = 0 means no estimate this time.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition

from distance_estimator import ground_distance, px4_roll_pitch_down, camera_height

MAX_AGE = 0.3     # seconds: ignore PX4 data older than this
NAN = float('nan')


class DistanceEstimator(Node):
    def __init__(self):
        super().__init__('distance_estimator')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.on_att, qos)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.on_pos, qos)
        self.create_subscription(Float32MultiArray, '/target/bbox', self.on_bbox, 10)
        self.pub = self.create_publisher(Float32MultiArray, '/target/estimate', 10)
        self.roll = self.pitch_down = self.alt = None
        self.att_t = self.pos_t = None
        self.count = 0
        self.get_logger().info('Distance estimator ready. Waiting for the detector and PX4...')

    def age(self, t):
        return (self.get_clock().now() - t).nanoseconds / 1e9 if t is not None else 1e9

    def on_att(self, msg):
        self.roll, self.pitch_down = px4_roll_pitch_down(list(msg.q))
        self.att_t = self.get_clock().now()

    def on_pos(self, msg):
        self.alt = -msg.z                 # NED: z is negative upwards
        self.pos_t = self.get_clock().now()

    def publish(self, vals):
        out = Float32MultiArray()
        out.data = [float(v) for v in vals]
        self.pub.publish(out)

    def on_bbox(self, msg):
        d = msg.data
        if len(d) < 6 or d[0] < 0.5 or self.age(self.att_t) > MAX_AGE or self.age(self.pos_t) > MAX_AGE:
            self.publish([0.0] + [NAN] * 7)
            return
        cx, cy, h = d[1], d[2], d[4]
        cam_h = camera_height(self.alt, self.roll, self.pitch_down)
        g = ground_distance(cx, cy + h / 2, cam_h, self.roll, self.pitch_down, 0.0)
        if g is None:                      # box bottom is above the horizon
            self.publish([0.0] + [NAN] * 7)
            return
        self.publish([1.0, g['range'], g['ground'], g['fwd'], g['lat'], cam_h,
                      math.degrees(self.roll), math.degrees(self.pitch_down)])
        self.count += 1
        if self.count % 20 == 1:
            self.get_logger().info(
                f'range={g["range"]:.2f} m  ground={g["ground"]:.2f} m  fwd={g["fwd"]:.2f}  lat={g["lat"]:+.2f}  '
                f'| cam height={cam_h:.2f} m  pitch(down)={math.degrees(self.pitch_down):+.1f} deg')


def main():
    rclpy.init()
    node = DistanceEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()