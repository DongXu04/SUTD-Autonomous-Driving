#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info("Teleop Node Started. Use WASD keys to move. Ctrl+C to quit.")

    def get_key(self):
        """Reads a single keypress from stdin."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        twist = Twist()
        while rclpy.ok():
            key = self.get_key()
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            if key.lower() == 'w':
                twist.linear.x = 1.0
            elif key.lower() == 's':
                twist.linear.x = -1.0
            elif key.lower() == 'a':
                twist.angular.z = 1.0
            elif key.lower() == 'd':
                twist.angular.z = -1.0
            elif key == '\x03':  # Ctrl+C
                break

            self.pub.publish(twist)
            self.get_logger().info(f"Published: linear.x={twist.linear.x}, angular.z={twist.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
