#!/usr/bin/env python3
"""AeroScout X450 - first ROS 2 flight: arm, take off to 5 m, hover 10 s, land."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)

TARGET_ALT = 5.0      # metres above the start point
HOLD_SECONDS = 10.0   # time to hover before landing
ARMED = 2             # VehicleStatus.arming_state value for "armed"
OFFBOARD = 14         # VehicleStatus.nav_state value for "offboard"


class OffboardTakeoff(Node):
    def __init__(self):
        super().__init__('offboard_takeoff')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.sp_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command_v1', qos)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.on_position, qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v5', self.on_status, qos)

        self.z = 0.0                # NED: negative z means up
        self.arming_state = 0
        self.nav_state = 0
        self.ticks = 0
        self.state = 'WARMUP'       # WARMUP -> CLIMB -> HOLD -> LAND -> DONE
        self.hold_start = None
        self.timer = self.create_timer(0.1, self.loop)   # 10 Hz
        self.get_logger().info('Node started. Waiting for PX4 data...')

    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def on_position(self, msg):
        self.z = msg.z

    def on_status(self, msg):
        self.arming_state = msg.arming_state
        self.nav_state = msg.nav_state

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

    def arm_and_offboard(self):
        self.send_command(176, 1.0, 6.0)   # DO_SET_MODE: custom mode, offboard
        self.send_command(400, 1.0)        # COMPONENT_ARM_DISARM: arm

    def loop(self):
        self.ticks += 1

        # The offboard heartbeat and a setpoint must be sent all the time.
        mode = OffboardControlMode()
        mode.timestamp = self.now_us()
        mode.position = True
        mode.velocity = False
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        self.mode_pub.publish(mode)

        sp = TrajectorySetpoint()
        sp.timestamp = self.now_us()
        target_z = -TARGET_ALT if self.state in ('CLIMB', 'HOLD', 'WARMUP') else 0.0
        sp.position = [0.0, 0.0, float(target_z)]
        sp.yaw = 0.0
        self.sp_pub.publish(sp)

        if self.state == 'WARMUP':
            # after 1 s of heartbeat, ask for offboard + arm, and repeat until it works
            if self.ticks >= 10 and self.ticks % 10 == 0:
                if not (self.arming_state == ARMED and self.nav_state == OFFBOARD):
                    self.arm_and_offboard()
            if self.arming_state == ARMED and self.nav_state == OFFBOARD:
                self.get_logger().info('Armed, in offboard mode. Climbing...')
                self.state = 'CLIMB'

        elif self.state == 'CLIMB':
            if self.ticks % 10 == 0:
                self.get_logger().info(f'Altitude {-self.z:.1f} m')
            if self.z <= -(TARGET_ALT - 0.3):
                self.get_logger().info('Target altitude reached. Hovering...')
                self.hold_start = self.get_clock().now()
                self.state = 'HOLD'

        elif self.state == 'HOLD':
            held = (self.get_clock().now() - self.hold_start).nanoseconds / 1e9
            if held >= HOLD_SECONDS:
                self.get_logger().info('Hold finished. Landing...')
                self.send_command(21)          # NAV_LAND
                self.state = 'LAND'

        elif self.state == 'LAND':
            if self.ticks % 10 == 0:
                self.get_logger().info(f'Altitude {-self.z:.1f} m')
            if self.arming_state != ARMED:
                self.get_logger().info('Landed and disarmed. Done.')
                self.state = 'DONE'
                self.timer.cancel()
                rclpy.shutdown()


def main():
    rclpy.init()
    node = OffboardTakeoff()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
