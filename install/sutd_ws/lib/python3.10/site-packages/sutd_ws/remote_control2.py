#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np


class JoyCarControl(Node):
    def __init__(self):
        super().__init__('joy_car_control')

        # Publisher to the /drive topic
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # Subscriber to /joy topic
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Car state
        self.speed = 0.0
        self.steering = 0.0

        # Config
        self.max_speed = 4.0
        self.max_steering = 0.20

        # Logitech F710 mapping (example indices)
        self.axis_throttle = 1   # Left stick vertical
        self.axis_steering = 3   # Right stick horizontal
        self.button_stop = 0     # A button
        self.button_quit = 1     # B button

        self.get_logger().info("JoyCarControl ready. Left stick = throttle, Right stick = steering.")

    def joy_callback(self, msg: Joy):
        # Ensure axis indices exist
        if len(msg.axes) <= max(self.axis_throttle, self.axis_steering):
            self.get_logger().warn("Joystick message missing expected axes.")
            return

        # Read joystick inputs
        throttle_input = msg.axes[self.axis_throttle]
        steering_input = msg.axes[self.axis_steering]

        # Invert throttle (forward stick = positive speed)
        self.speed = np.clip(throttle_input * self.max_speed, -self.max_speed, self.max_speed)
        self.steering = np.clip(steering_input * self.max_steering, -self.max_steering, self.max_steering)

        # Stop button
        if len(msg.buttons) > self.button_stop and msg.buttons[self.button_stop]:
            self.speed = 0.0
            self.steering = 0.0
            self.get_logger().info("Emergency stop triggered.")

        # Quit button
        if len(msg.buttons) > self.button_quit and msg.buttons[self.button_quit]:
            self.get_logger().info("Quit button pressed. Shutting down node.")
            rclpy.shutdown()
            return

        # Publish Ackermann command
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = float(self.speed)
        drive_msg.drive.steering_angle = float(self.steering)
        self.drive_pub.publish(drive_msg)

        self.get_logger().info(f"Speed: {self.speed:.2f}, Steering: {self.steering:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = JoyCarControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
