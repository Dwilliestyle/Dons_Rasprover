#!/usr/bin/env python3
"""
color_tracker.py
----------------
Captures frames from a USB webcam, finds the largest blob matching
a configured HSV color range, and publishes tracking data for the
follow_controller to consume.

Published Topics:
  /rasprover/tracker/target   (geometry_msgs/Point)
      x = horizontal pixel offset from frame center (-320 to +320)
      y = vertical pixel offset from frame center (unused by controller)
      z = contour area in pixels² (used as a proxy for distance)

  /rasprover/tracker/debug_image  (sensor_msgs/Image)  [optional]
      Annotated frame showing the detection circle and crosshair

Parameters (from tracking_params.yaml):
  camera_index, camera_width, camera_height
  h_min, h_max, s_min, s_max, v_min, v_max
  min_contour_area
  publish_debug_image
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ColorTracker(Node):

    def __init__(self):
        super().__init__('color_tracker')

        # --- Parameters ---
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('h_min', 5)
        self.declare_parameter('h_max', 20)
        self.declare_parameter('s_min', 150)
        self.declare_parameter('s_max', 255)
        self.declare_parameter('v_min', 70)
        self.declare_parameter('v_max', 255)
        self.declare_parameter('min_contour_area', 500)
        self.declare_parameter('publish_debug_image', True)

        cam_idx   = self.get_parameter('camera_index').value
        self.width  = self.get_parameter('camera_width').value
        self.height = self.get_parameter('camera_height').value
        self.publish_debug = self.get_parameter('publish_debug_image').value

        # Build HSV bounds arrays
        self._update_hsv_bounds()

        # --- Camera ---
        self.cap = cv2.VideoCapture(cam_idx)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        if not self.cap.isOpened():
            self.get_logger().error(
                f'Could not open camera at index {cam_idx}. '
                'Check that the USB webcam is connected and try index 1 if 0 fails.'
            )
            raise RuntimeError('Camera not available')

        self.get_logger().info(
            f'Camera opened at index {cam_idx} ({self.width}x{self.height})'
        )

        # --- Publishers ---
        self.target_pub = self.create_publisher(Point, '/rasprover/tracker/target', 10)

        self.bridge = CvBridge()
        if self.publish_debug:
            self.debug_pub = self.create_publisher(
                Image, '/rasprover/tracker/debug_image', 10
            )

        # --- Timer: run at ~30 Hz ---
        self.timer = self.create_timer(1.0 / 30.0, self.process_frame)

        self.get_logger().info('ColorTracker node started. Tracking active.')

    # ------------------------------------------------------------------
    def _update_hsv_bounds(self):
        """Read HSV params and build numpy arrays."""
        h_min = self.get_parameter('h_min').value
        h_max = self.get_parameter('h_max').value
        s_min = self.get_parameter('s_min').value
        s_max = self.get_parameter('s_max').value
        v_min = self.get_parameter('v_min').value
        v_max = self.get_parameter('v_max').value
        self.lower_hsv = np.array([h_min, s_min, v_min], dtype=np.uint8)
        self.upper_hsv = np.array([h_max, s_max, v_max], dtype=np.uint8)

    # ------------------------------------------------------------------
    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame from camera.')
            return

        min_area = self.get_parameter('min_contour_area').value
        cx = self.width  // 2
        cy = self.height // 2

        # --- Color detection pipeline ---
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask    = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)

        # Morphological clean-up: fill holes, remove speckles
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        target_msg = Point()  # default all zeros = "no target"
        detected   = False

        if contours:
            # Pick the largest contour
            largest = max(contours, key=cv2.contourArea)
            area    = cv2.contourArea(largest)

            if area >= min_area:
                detected = True
                # Centroid via moments
                M    = cv2.moments(largest)
                tx   = int(M['m10'] / M['m00'])
                ty   = int(M['m01'] / M['m00'])

                # Publish error from frame center and area
                target_msg.x = float(tx - cx)   # + = right of center
                target_msg.y = float(ty - cy)   # + = below center
                target_msg.z = float(area)

                if self.publish_debug:
                    # Draw enclosing circle + crosshair
                    (bx, by), radius = cv2.minEnclosingCircle(largest)
                    cv2.circle(frame, (int(bx), int(by)), int(radius),
                               (0, 255, 0), 2)
                    cv2.circle(frame, (tx, ty), 5, (0, 0, 255), -1)
                    cv2.putText(
                        frame,
                        f'err_x:{target_msg.x:.0f}  area:{area:.0f}',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 0), 2
                    )

        # Always publish (zeros = no target, controller handles it)
        self.target_pub.publish(target_msg)

        if self.publish_debug:
            # Draw center crosshair
            cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (255, 255, 0), 1)
            cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (255, 255, 0), 1)
            if not detected:
                cv2.putText(frame, 'NO TARGET', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            self.debug_pub.publish(
                self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            )

    # ------------------------------------------------------------------
    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ColorTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()