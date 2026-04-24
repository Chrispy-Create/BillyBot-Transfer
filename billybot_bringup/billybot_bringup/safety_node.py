#!/usr/bin/env python3
"""
BillyBot safety node — obstacle stop.

Sits between the web controller and the motor controller:
  /cmd_vel_raw  (web controller output)  →  safety check  →  /cmd_vel  (motors)

When any sensor in the configured forward arc reads below STOP_DISTANCE the
node replaces forward motion with a zero-velocity command.  Backing up is
always allowed so the robot can self-recover.  Hysteresis (RESUME_DISTANCE)
prevents rapid on/off cycling.

If the Arduino/ultrasonic bridge is not running, the node passes cmd_vel
straight through (fail-open), so the robot still drives normally without
ultrasonic hardware connected.

Subscribes:
  /cmd_vel_raw              geometry_msgs/Twist   (from web controller)
  /ultrasonic/<name>        sensor_msgs/Range      (from ultrasonic_bridge)

Publishes:
  /cmd_vel                  geometry_msgs/Twist   (to motor controller / Pico)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

# ── Sensor names that face the forward arc ────────────────────────────────────
# Adjust this list if your sensor layout changes.
FRONT_SENSORS = [
    'low_left',        'low_center_left',  'low_center_right',  'low_right',
    'mid_left',        'mid_center_left',  'mid_center_right',  'mid_right',
    'high_left',       'high_center_left', 'high_center_right', 'high_right',
]

STOP_DISTANCE   = 0.50   # m — stop if any front sensor reads closer than this
RESUME_DISTANCE = 0.70   # m — allow forward motion again once all clear of this


class SafetyNode(Node):
    def __init__(self):
        super().__init__('billybot_safety')

        self._blocked = False
        self._distances = {name: float('inf') for name in FRONT_SENSORS}

        # Motor output
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Web-controller input (renamed so safety node is the sole /cmd_vel publisher)
        self.create_subscription(Twist, 'cmd_vel_raw', self._cmd_vel_cb, 10)

        # One subscription per ultrasonic sensor
        for name in FRONT_SENSORS:
            self.create_subscription(
                Range,
                f'ultrasonic/{name}',
                lambda msg, n=name: self._range_cb(msg, n),
                10
            )

        self.get_logger().info(
            f'Safety node active — stop < {STOP_DISTANCE} m, '
            f'resume > {RESUME_DISTANCE} m'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _range_cb(self, msg: Range, sensor_name: str):
        self._distances[sensor_name] = msg.range
        min_dist = min(self._distances.values())

        if min_dist < STOP_DISTANCE:
            if not self._blocked:
                self.get_logger().warn(
                    f'Obstacle! Nearest sensor ({sensor_name}) = {min_dist:.2f} m'
                )
            self._blocked = True
        elif min_dist > RESUME_DISTANCE:
            if self._blocked:
                self.get_logger().info('Path clear — resuming.')
            self._blocked = False

    def _cmd_vel_cb(self, msg: Twist):
        if self._blocked and msg.linear.x > 0.0:
            # Block forward drive; let the operator back out
            self._cmd_pub.publish(Twist())
        else:
            self._cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
