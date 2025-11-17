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

        # PID parameters (well… PD, because someone didn’t ask for I)
        self.desired_distance = 0
        self.kp = 0.8
        self.kd = 0.3       # derivative gain, adjust before blaming me when it oscillates

        self.speed = 2.5

        # keep track of previous error for derivative term
        self.prev_error = 0.0
        self.prev_time = None

    def scan_callback(self, scan):
        right_angle = -90 * np.pi / 180
        left_angle = 90 * np.pi / 180

        right_index = int((right_angle - scan.angle_min) / scan.angle_increment)
        left_index = int((left_angle - scan.angle_min) / scan.angle_increment)

        right_index = max(0, min(right_index, len(scan.ranges) - 1))
        left_index = max(0, min(left_index, len(scan.ranges) - 1))

        right_dist = scan.ranges[right_index]
        left_dist = scan.ranges[left_index]

        if (np.isinf(right_dist) or np.isnan(right_dist) or
            np.isinf(left_dist) or np.isnan(left_dist)):
            self.get_logger().warn("Invalid scan data, skipping frame.")
            return

        # error: left minus right
        midpoint_error = left_dist - right_dist

        # time for derivative calc
        current_time = self.get_clock().now().nanoseconds * 1e-9

        if self.prev_time is None:
            # first frame, no derivative possible
            derivative = 0.0
        else:
            dt = current_time - self.prev_time
            if dt <= 0:
                derivative = 0.0
            else:
                derivative = (midpoint_error - self.prev_error) / dt

        # PD controller output
        steering_angle = self.kp * midpoint_error + self.kd * derivative

        # save for next iteration
        self.prev_error = midpoint_error
        self.prev_time = current_time

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
