#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class KeyboardListener(Node):
    def __init__(self):
        super().__init__('keyboard_listener')
        # topic name, msg type, callback, QoS depth
        self.sub = self.create_subscription(
            String,
            'key_input',
            self.callback,
            10)
        self.sub  # avoid unused var warning
        self.get_logger().info('Keyboard listener started. Waiting for key_input...')

    def callback(self, msg):
        k = msg.data
        if k == 'w':
            print("Moving forward")
        elif k == 's':
            print("Moving backward")
        elif k == 'a':
            print("Turning left")
        elif k == 'd':
            print("Turning right")
        else:
            print("Received key:", repr(k))


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
