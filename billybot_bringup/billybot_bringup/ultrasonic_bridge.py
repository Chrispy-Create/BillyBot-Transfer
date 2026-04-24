#!/usr/bin/env python3
"""
Ultrasonic bridge node.

Reads distance packets from the Arduino Mega (HC-SR04 × 12) over USB serial
and publishes each sensor as a sensor_msgs/Range topic.

Serial format from Arduino: US:d0:d1:d2:d3:d4:d5:d6:d7:d8:d9:d10:d11\n
where d0-d11 are distances in metres (float).

Sensor layout (4 per height level):
  low_left  low_center_left  low_center_right  low_right
  mid_left  mid_center_left  mid_center_right  mid_right
  high_left high_center_left high_center_right high_right

Publishes: /ultrasonic/<sensor_name>  (sensor_msgs/Range)
"""

import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

import serial

SENSOR_NAMES = [
    'low_left',         'low_center_left',   'low_center_right',   'low_right',
    'mid_left',         'mid_center_left',   'mid_center_right',   'mid_right',
    'high_left',        'high_center_left',  'high_center_right',  'high_right',
]

HC_SR04_FOV      = 0.26   # ~15 degrees (radians)
HC_SR04_MIN_M    = 0.02   # 2 cm
HC_SR04_MAX_M    = 4.0    # 400 cm


class UltrasonicBridge(Node):
    def __init__(self):
        super().__init__('ultrasonic_bridge')

        self.declare_parameter('serial_port', '/dev/ttyACM1')
        self.declare_parameter('baud_rate',   115200)

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value

        self._pubs = {
            name: self.create_publisher(Range, f'ultrasonic/{name}', 10)
            for name in SENSOR_NAMES
        }

        try:
            self._ser = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f'Arduino connected on {port} @ {baud} baud')
        except serial.SerialException as exc:
            self.get_logger().error(f'Cannot open {port}: {exc}')
            self._ser = None
            return

        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        while rclpy.ok():
            if self._ser is None:
                return
            try:
                raw = self._ser.readline()
                line = raw.decode('utf-8', errors='ignore').strip()
            except Exception as exc:
                self.get_logger().warn(f'Serial read error: {exc}', throttle_duration_sec=5.0)
                continue

            if not line.startswith('US:'):
                continue

            parts = line[3:].split(':')
            if len(parts) != len(SENSOR_NAMES):
                continue

            now = self.get_clock().now().to_msg()
            for name, raw_val in zip(SENSOR_NAMES, parts):
                try:
                    dist = float(raw_val)
                except ValueError:
                    continue

                msg = Range()
                msg.header.stamp       = now
                msg.header.frame_id    = f'ultrasonic_{name}'
                msg.radiation_type     = Range.ULTRASOUND
                msg.field_of_view      = HC_SR04_FOV
                msg.min_range          = HC_SR04_MIN_M
                msg.max_range          = HC_SR04_MAX_M
                # Out-of-range readings come back as 4.5 from Arduino; clamp to inf
                msg.range = dist if HC_SR04_MIN_M <= dist <= HC_SR04_MAX_M else float('inf')
                self._pubs[name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
