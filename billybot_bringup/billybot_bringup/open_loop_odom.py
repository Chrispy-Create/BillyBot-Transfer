#!/usr/bin/env python3
"""
Open-loop odometry node.

Integrates the commanded velocity (cmd_vel) to produce an odom→base_link TF
and nav_msgs/Odometry message.  This gives SLAM Toolbox the odom frame it
needs while wheel-encoder odometry is not yet available.

Once the hall-sensor differential receiver arrives and is wired up, replace
this node with a proper encoder-based odometry publisher subscribed to the
Pico's hall-sensor topic.

Subscribes : /cmd_vel        (geometry_msgs/Twist)
Publishes  : /odom           (nav_msgs/Odometry)
TF broadcast: odom → base_link
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class OpenLoopOdom(Node):
    def __init__(self):
        super().__init__('open_loop_odom')

        self._x     = 0.0
        self._y     = 0.0
        self._theta = 0.0
        self._vx    = 0.0
        self._vth   = 0.0
        self._last  = self.get_clock().now()

        self._tf_br   = TransformBroadcaster(self)
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)

        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_cb, 10)

        # Publish at 20 Hz — fast enough for SLAM Toolbox
        self.create_timer(0.05, self._publish)

    def _cmd_vel_cb(self, msg: Twist):
        self._vx  = msg.linear.x
        self._vth = msg.angular.z

    def _publish(self):
        now = self.get_clock().now()
        dt  = (now - self._last).nanoseconds * 1e-9
        self._last = now

        # Integrate
        self._theta += self._vth * dt
        self._x     += self._vx * math.cos(self._theta) * dt
        self._y     += self._vx * math.sin(self._theta) * dt

        # Quaternion from yaw only
        sy = math.sin(self._theta * 0.5)
        cy = math.cos(self._theta * 0.5)

        # ── TF broadcast ──────────────────────────────────────────────────────
        t = TransformStamped()
        t.header.stamp       = now.to_msg()
        t.header.frame_id    = 'odom'
        t.child_frame_id     = 'base_link'
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.translation.z = 0.0
        t.transform.rotation.x    = 0.0
        t.transform.rotation.y    = 0.0
        t.transform.rotation.z    = sy
        t.transform.rotation.w    = cy
        self._tf_br.sendTransform(t)

        # ── Odometry message ──────────────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp         = now.to_msg()
        odom.header.frame_id      = 'odom'
        odom.child_frame_id       = 'base_link'
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy
        odom.twist.twist.linear.x    = self._vx
        odom.twist.twist.angular.z   = self._vth
        self._odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
