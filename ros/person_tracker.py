#!/usr/bin/env python3
"""AeroScout X450 - person tracker.

Takes off to ALT metres, then keeps the person from /target/bbox centred in the
camera image (by turning) and at a steady distance (by moving forward/back).
Lands after TRACK_SECONDS, or when you press Ctrl+C.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)

NAN = float('nan')
ALT = 1.5             # flying height, metres
H_TARGET = 0.35       # wanted person height as a fraction of the image (bigger = closer)
K_YAW = 2.8           # turn rate (rad/s) per full image width of error
K_FWD = 3.0           # forward speed (m/s) per unit of height error
K_ALT = 1.0
MAX_YAW_RATE = 0.8    # rad/s
MAX_FWD = 1.2         # m/s
MAX_BACK = 0.6        # m/s
SEARCH_YAW_RATE = 0.3 # rad/s, slow turn when the person is lost
LOST_AFTER = 0.7      # seconds without a fresh detection = lost
SEARCH_AFTER = 2.0    # seconds lost before starting to turn
TRACK_SECONDS = 300.0
ARMED = 2
OFFBOARD = 14


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class PersonTracker(Node):
    def __init__(self):
        super().__init__('person_tracker')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.sp_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command_v1', qos)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.on_position, qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v5', self.on_status, qos)
        self.create_subscription(Float32MultiArray, '/target/bbox', self.on_bbox, 10)

        self.z = 0.0
        self.heading = 0.0
        self.arming_state = 0
        self.nav_state = 0
        self.cx = 0.5
        self.h = H_TARGET
        self.last_seen = None            # time of the last real detection
        self.ticks = 0
        self.state = 'WARMUP'            # WARMUP -> CLIMB -> TRACK -> LAND
        self.track_start = None
        self.timer = self.create_timer(0.05, self.loop)   # 20 Hz
        self.get_logger().info('Tracker started. Waiting for PX4 data...')

    # ---- helpers -------------------------------------------------------
    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def seconds_since(self, t):
        return (self.get_clock().now() - t).nanoseconds / 1e9

    def on_position(self, msg):
        self.z = msg.z
        self.heading = msg.heading

    def on_status(self, msg):
        self.arming_state = msg.arming_state
        self.nav_state = msg.nav_state

    def on_bbox(self, msg):
        d = msg.data
        if len(d) >= 6 and d[0] > 0.5:
            a = 0.5   # smoothing: 0 = ignore new data, 1 = no smoothing
            self.cx = (1 - a) * self.cx + a * d[1]
            self.h = (1 - a) * self.h + a * d[4]
            self.last_seen = self.get_clock().now()

    def send_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.now_us()
        msg.command = command
        msg.param1 = float(p1)
        msg.param2 = float(p2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    def publish_setpoint(self, position_mode, pos=None, vel=None, yawspeed=NAN):
        mode = OffboardControlMode()
        mode.timestamp = self.now_us()
        mode.position = position_mode
        mode.velocity = not position_mode
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        self.mode_pub.publish(mode)

        sp = TrajectorySetpoint()
        sp.timestamp = self.now_us()
        sp.position = pos if pos is not None else [NAN, NAN, NAN]
        sp.velocity = vel if vel is not None else [NAN, NAN, NAN]
        sp.acceleration = [NAN, NAN, NAN]
        sp.jerk = [NAN, NAN, NAN]
        sp.yaw = NAN
        sp.yawspeed = float(yawspeed)
        self.sp_pub.publish(sp)

    # ---- main loop -----------------------------------------------------
    def loop(self):
        self.ticks += 1

        if self.state in ('WARMUP', 'CLIMB'):
            self.publish_setpoint(True, pos=[0.0, 0.0, -ALT])
        elif self.state == 'LAND':
            self.publish_setpoint(True, pos=[0.0, 0.0, 0.0])

        if self.state == 'WARMUP':
            if self.ticks >= 20 and self.ticks % 20 == 0:
                if not (self.arming_state == ARMED and self.nav_state == OFFBOARD):
                    self.send_command(176, 1.0, 6.0)   # offboard mode
                    self.send_command(400, 1.0)        # arm
            if self.arming_state == ARMED and self.nav_state == OFFBOARD:
                self.get_logger().info('Armed, in offboard mode. Climbing...')
                self.state = 'CLIMB'

        elif self.state == 'CLIMB':
            if self.z <= -(ALT - 0.2):
                self.get_logger().info('At flying height. Tracking started.')
                self.track_start = self.get_clock().now()
                self.state = 'TRACK'

        elif self.state == 'TRACK':
            visible = (self.last_seen is not None and
                       self.seconds_since(self.last_seen) < LOST_AFTER)
            if visible:
                yawspeed = clamp(K_YAW * (self.cx - 0.5), -MAX_YAW_RATE, MAX_YAW_RATE)
                fwd = clamp(K_FWD * (H_TARGET - self.h), -MAX_BACK, MAX_FWD)
            else:
                lost_for = (self.seconds_since(self.last_seen)
                            if self.last_seen is not None else 99.0)
                yawspeed = SEARCH_YAW_RATE if lost_for > SEARCH_AFTER else 0.0
                fwd = 0.0
            vz = clamp(K_ALT * (-ALT - self.z), -0.8, 0.8)
            vx = fwd * math.cos(self.heading)    # NED: x = north, y = east
            vy = fwd * math.sin(self.heading)
            self.publish_setpoint(False, vel=[vx, vy, vz], yawspeed=yawspeed)

            if self.ticks % 20 == 0:
                self.get_logger().info(
                    f'{"person" if visible else "LOST  "} cx={self.cx:.2f} h={self.h:.2f} '
                    f'fwd={fwd:+.2f} m/s turn={yawspeed:+.2f} rad/s alt={-self.z:.1f} m')

            if self.seconds_since(self.track_start) >= TRACK_SECONDS:
                self.get_logger().info('Time is up. Landing...')
                self.land()

        elif self.state == 'LAND':
            if self.ticks % 20 == 0:
                self.get_logger().info(f'Landing, alt={-self.z:.1f} m')
            if self.arming_state != ARMED:
                self.get_logger().info('Landed and disarmed. Done.')
                self.timer.cancel()
                rclpy.shutdown()

    def land(self):
        self.send_command(21)    # NAV_LAND
        self.state = 'LAND'


def main():
    rclpy.init()
    node = PersonTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info('Ctrl+C: landing...')
            node.land()
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
