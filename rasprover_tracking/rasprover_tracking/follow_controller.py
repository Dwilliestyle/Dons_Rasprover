#!/usr/bin/env python3
"""
follow_controller.py
--------------------
Subscribes to the color tracker's target data and drives the RasPRover
to follow the detected object using a proportional controller.

States:
  TRACKING  — target visible, steering toward it and matching distance
  SEARCHING — target lost, slowly rotating to find it again
  IDLE      — search timed out, robot stopped

Subscribed Topics:
  /rasprover/tracker/target   (geometry_msgs/Point)
      x = horizontal error (pixels), z = target area (pixels²)

Published Topics:
  /cmd_vel   (geometry_msgs/Twist)

Parameters (from tracking_params.yaml):
  kp_angular, kp_linear, target_area
  max_linear_speed, max_angular_speed
  search_angular_speed, search_timeout
  angular_dead_zone, area_dead_zone
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
import math


class FollowState:
    TRACKING  = 'TRACKING'
    SEARCHING = 'SEARCHING'
    IDLE      = 'IDLE'


class FollowController(Node):

    def __init__(self):
        super().__init__('follow_controller')

        # --- Parameters ---
        self.declare_parameter('kp_angular',          0.003)
        self.declare_parameter('kp_linear',           0.00005)
        self.declare_parameter('target_area',         8000.0)
        self.declare_parameter('max_linear_speed',    0.25)
        self.declare_parameter('max_angular_speed',   0.8)
        self.declare_parameter('search_angular_speed',0.4)
        self.declare_parameter('search_timeout',      5.0)
        self.declare_parameter('angular_dead_zone',   20.0)
        self.declare_parameter('area_dead_zone',      500.0)

        # --- State ---
        self.state             = FollowState.IDLE
        self.last_seen_time    = None
        self.last_target_x     = 0.0  # remember which way to search

        # --- ROS interfaces ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            Point,
            '/rasprover/tracker/target',
            self.target_callback,
            10
        )

        # Control loop at 20 Hz
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info('FollowController started. Waiting for target...')

    # ------------------------------------------------------------------
    def target_callback(self, msg: Point):
        """Called every time the tracker publishes a new detection."""
        # A zero Point means no target detected
        if msg.x == 0.0 and msg.y == 0.0 and msg.z == 0.0:
            # Don't update last_seen_time — let it age out naturally
            return

        # Valid target received
        self.last_seen_time = self.get_clock().now()
        self.last_target_x  = msg.x
        self._latest_target = msg

        if self.state != FollowState.TRACKING:
            self.get_logger().info('Target acquired — TRACKING')
            self.state = FollowState.TRACKING

    # ------------------------------------------------------------------
    def control_loop(self):
        """Runs at 20 Hz. Decides what velocity to publish."""
        twist = Twist()
        now   = self.get_clock().now()

        # --- Check for target timeout ---
        if self.last_seen_time is not None:
            elapsed = (now - self.last_seen_time).nanoseconds / 1e9
            search_timeout = self.get_parameter('search_timeout').value
            if elapsed > 0.15 and self.state == FollowState.TRACKING:
                self.get_logger().info('Target lost — SEARCHING')
                self.state = FollowState.SEARCHING
            if elapsed > search_timeout and self.state == FollowState.SEARCHING:
                self.get_logger().info('Search timed out — IDLE')
                self.state = FollowState.IDLE

        # --- Act based on state ---
        if self.state == FollowState.TRACKING:
            twist = self._compute_tracking_twist()

        elif self.state == FollowState.SEARCHING:
            twist = self._compute_search_twist()

        # IDLE: twist is all zeros — robot stops

        self.cmd_pub.publish(twist)

    # ------------------------------------------------------------------
    def _compute_tracking_twist(self) -> Twist:
        """Proportional controller for turning and distance."""
        kp_ang  = self.get_parameter('kp_angular').value
        kp_lin  = self.get_parameter('kp_linear').value
        t_area  = self.get_parameter('target_area').value
        max_lin = self.get_parameter('max_linear_speed').value
        max_ang = self.get_parameter('max_angular_speed').value
        adz     = self.get_parameter('angular_dead_zone').value
        ldz     = self.get_parameter('area_dead_zone').value

        msg = self._latest_target
        err_x    = msg.x    # pixels: + = target is right of center
        err_area = t_area - msg.z  # + = too far (need to drive forward)

        # Apply dead zones
        if abs(err_x) < adz:
            err_x = 0.0
        if abs(err_area) < ldz:
            err_area = 0.0

        # Proportional outputs
        # Negate err_x: target right → turn right (negative angular in ROS)
        angular_z = -kp_ang * err_x
        linear_x  =  kp_lin * err_area

        # Clamp to limits
        angular_z = max(-max_ang, min(max_ang, angular_z))
        linear_x  = max(-max_lin, min(max_lin, linear_x))

        twist = Twist()
        twist.linear.x  = linear_x
        twist.angular.z = angular_z
        return twist

    # ------------------------------------------------------------------
    def _compute_search_twist(self) -> Twist:
        """Rotate slowly in the direction the target was last seen."""
        speed = self.get_parameter('search_angular_speed').value

        # Turn toward where we last saw it
        # last_target_x > 0 means target was to the right → turn right (negative z)
        direction = -1.0 if self.last_target_x >= 0 else 1.0

        twist = Twist()
        twist.angular.z = direction * speed
        return twist

    # ------------------------------------------------------------------
    def _stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = FollowController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()