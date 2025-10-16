#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class MidpointFollower(Node):
    def __init__(self):
        super().__init__('midpoint_follower')

        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.desired_distance = 0.8   # target distance to both walls (so midpoint target)
        self.kp = 1.2                 # proportional gain
        self.speed = 2.5              # forward speed

    def scan_callback(self, scan):
        # Angles for left and right walls (~90 deg left/right)
        right_angle = -90 * np.pi / 180
        left_angle = 90 * np.pi / 180

        # Convert angles to indices
        right_index = int((right_angle - scan.angle_min) / scan.angle_increment)
        left_index = int((left_angle - scan.angle_min) / scan.angle_increment)

        # Clamp indices within scan range
        right_index = max(0, min(right_index, len(scan.ranges) - 1))
        left_index = max(0, min(left_index, len(scan.ranges) - 1))

        # Get distances
        right_dist = scan.ranges[right_index]
        left_dist = scan.ranges[left_index]

        # Ignore inf/nan readings
        if np.isinf(right_dist) or np.isnan(right_dist) or np.isinf(left_dist) or np.isnan(left_dist):
            self.get_logger().warn("Invalid scan data, skipping frame.")
            return

        # Compute midpoint error (difference between left and right)
        midpoint_error = left_dist - right_dist

        # Proportional control to center between walls
        steering_angle = self.kp * midpoint_error

        # Publish drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.speed
        drive_msg.drive.steering_angle = steering_angle
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MidpointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
