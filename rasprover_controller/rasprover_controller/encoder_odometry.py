#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


class EncoderOdometry(Node):
    """
    Encoder-based Odometry for Skid-Steer RaspRover
    
    Subscribes to:
        - /odom/odom_raw (std_msgs/Float32MultiArray): [left_encoder, right_encoder] in meters
    
    Publishes to:
        - /odom (nav_msgs/Odometry): Robot odometry based on encoder feedback
        - TF: odom -> base_footprint transform
    """
    
    def __init__(self):
        super().__init__('encoder_odometry')
        
        # Declare parameters
        self.declare_parameter('wheel_track', 0.070)  # meters - distance between left and right wheel centers
        self.declare_parameter('skid_steer_factor', 1.0)  # Correction factor for skid-steer turning (tune empirically)
        self.declare_parameter('publish_tf', True)  # Whether to publish TF transforms
        
        # Get parameters
        self.wheel_track = self.get_parameter('wheel_track').value
        self.skid_steer_factor = self.get_parameter('skid_steer_factor').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Previous encoder values
        self.left_encoder_prev = None
        self.right_encoder_prev = None
        self.last_time = None
        
        # Current velocities (for publishing in odom message)
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # Subscriber
        self.encoder_sub = self.create_subscription(
            Float32MultiArray,
            'odom/odom_raw',
            self.encoder_callback,
            10
        )
        
        # Publisher
        self.odom_pub = self.create_publisher(
            Odometry,
            'odom',
            10
        )
        
        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('Encoder Odometry Node initialized')
        self.get_logger().info(f'Wheel track: {self.wheel_track}m')
        self.get_logger().info(f'Skid-steer correction factor: {self.skid_steer_factor}')
        self.get_logger().info(f'Publishing TF: {self.publish_tf}')
    
    def encoder_callback(self, msg):
        """Process encoder data and update odometry"""
        current_time = self.get_clock().now()
        
        # Extract encoder values (already in meters from ESP32 bridge)
        left_encoder = msg.data[0]
        right_encoder = msg.data[1]
        
        # Initialize on first reading
        if self.left_encoder_prev is None:
            self.left_encoder_prev = left_encoder
            self.right_encoder_prev = right_encoder
            self.last_time = current_time
            self.get_logger().info('Odometry initialized with first encoder reading')
            return
        
        # Calculate time delta
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0.0:
            self.get_logger().warn('Invalid time delta, skipping update')
            return
        
        # Calculate encoder deltas (distance traveled by each side)
        delta_left = left_encoder - self.left_encoder_prev
        delta_right = right_encoder - self.right_encoder_prev
        
        # Calculate forward distance (average of both sides)
        delta_distance = (delta_left + delta_right) / 2.0
        
        # Calculate change in heading (difference between sides divided by track width)
        # Apply skid-steer correction factor for turning (empirically tuned)
        delta_theta = ((delta_right - delta_left) / self.wheel_track) * self.skid_steer_factor
        
        # Update pose using midpoint integration
        # This assumes the robot travels in a small arc between measurements
        if abs(delta_theta) < 1e-6:
            # Straight line motion
            delta_x = delta_distance * math.cos(self.theta)
            delta_y = delta_distance * math.sin(self.theta)
        else:
            # Arc motion
            radius = delta_distance / delta_theta
            delta_x = radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            delta_y = -radius * (math.cos(self.theta + delta_theta) - math.cos(self.theta))
        
        # Update position
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Calculate velocities for odometry message
        self.linear_vel = delta_distance / dt
        self.angular_vel = delta_theta / dt
        
        # Publish odometry
        self.publish_odometry(current_time)
        
        # Update previous values
        self.left_encoder_prev = left_encoder
        self.right_encoder_prev = right_encoder
        self.last_time = current_time
    
    def publish_odometry(self, current_time):
        """Publish odometry message and TF transform"""
        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        
        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Velocity in body frame
        odom_msg.twist.twist.linear.x = self.linear_vel
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.angular_vel
        
        # Covariance matrices
        # Pose covariance (encoder-based odometry has moderate uncertainty)
        # These values should be tuned based on your robot's actual performance
        odom_msg.pose.covariance[0] = 0.01   # x variance (m^2)
        odom_msg.pose.covariance[7] = 0.01   # y variance (m^2)
        odom_msg.pose.covariance[14] = 1e6   # z variance (not used)
        odom_msg.pose.covariance[21] = 1e6   # roll variance (not used)
        odom_msg.pose.covariance[28] = 1e6   # pitch variance (not used)
        odom_msg.pose.covariance[35] = 0.05  # yaw variance (rad^2) - higher for skid-steer
        
        # Twist covariance
        odom_msg.twist.covariance[0] = 0.01   # vx variance
        odom_msg.twist.covariance[7] = 1e6    # vy variance (not used)
        odom_msg.twist.covariance[14] = 1e6   # vz variance (not used)
        odom_msg.twist.covariance[21] = 1e6   # vroll variance (not used)
        odom_msg.twist.covariance[28] = 1e6   # vpitch variance (not used)
        odom_msg.twist.covariance[35] = 0.02  # vyaw variance
        
        self.odom_pub.publish(odom_msg)
        
        # Publish TF transform if enabled
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(self.theta / 2.0)
            t.transform.rotation.w = math.cos(self.theta / 2.0)
            
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdometry()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()