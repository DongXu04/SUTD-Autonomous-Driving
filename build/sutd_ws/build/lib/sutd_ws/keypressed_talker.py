#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty
import select

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(String, 'key_input', 10)
        self.get_logger().info('Keyboard Publisher Started. Press q to quit.')

    def get_key(self):
        """Non-blocking key press detection"""
        dr, dw, de = select.select([sys.stdin], [], [], 0)
        if dr:
            return sys.stdin.read(1)
        return None

    def run(self):
        # Configure terminal to raw mode
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while rclpy.ok():
                key = self.get_key()
                if key:
                    if key == 'q':
                        self.get_logger().info('Quitting...')
                        break
                    msg = String()
                    msg.data = key
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Published: {key}')
                rclpy.spin_once(self, timeout_sec=0.01)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
