#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class KeyboardCarControl(Node):
    def __init__(self):
        super().__init__('keyboard_car_control')

        # Publisher to the simulator
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # Subscriber to keyboard inputs
        self.key_sub = self.create_subscription(String, '/key_input', self.key_callback, 10)

        # Car state
        self.speed = 0.0
        self.steering = 0.0

        # Config
        self.max_speed = 4.0
        self.min_speed = 0.0
        self.speed_step = 0.5
        self.steering_step = 0.075
        self.max_steering = 0.45

        self.get_logger().info("KeyboardCarControl ready. Use WASD keys to drive, Q to quit.")

    def key_callback(self, msg: String):
        key = msg.data.lower()

        if key == 'w':  # accelerate
            self.speed = min(self.max_speed, self.speed + self.speed_step)
        elif key == 's':  # decelerate / reverse
            self.speed = max(self.min_speed, self.speed - self.speed_step)
            
        elif key == 'a':  # steer left
            self.steering = np.clip(self.steering + self.steering_step, -self.max_steering, self.max_steering)
        elif key == 'd':  # steer right
            self.steering = np.clip(self.steering - self.steering_step, -self.max_steering, self.max_steering)
        elif key == 'x':  # stop
            self.speed = 0.0
            self.steering = 0.0
        elif key == 'q':  # quit signal
            self.get_logger().info('Q pressed. Shutting down node.')
            rclpy.shutdown()
            return
        else:
            # Ignore other keys
            return

        # Publish drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = float(self.speed)
        drive_msg.drive.steering_angle = float(self.steering)
        self.drive_pub.publish(drive_msg)
        self.get_logger().info(f'Speed: {self.speed:.2f}, Steering: {self.steering:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardCarControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()