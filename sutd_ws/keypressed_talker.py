#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys, termios, tty, select, atexit, signal, time, os


class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher_tty')
        self.pub = self.create_publisher(String, 'key_input', 10)
        self.get_logger().info('Publishing to /key_input. Press "q" to quit.')

        # Terminal setup
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        atexit.register(self.restore_terminal)

        self._running = True
        signal.signal(signal.SIGINT, self._sigint_handler)

    def _sigint_handler(self, signum, frame):
        self.shutdown_node()

    def restore_terminal(self):
        try:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        except Exception:
            pass

    def poll_stdin(self):
        """Poll keyboard input without blocking."""
        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        if not rlist:
            return
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            time.sleep(0.001)
            if select.select([sys.stdin], [], [], 0)[0]:
                ch += sys.stdin.read(2)

        label = self.human_readable(ch)
        msg = String()
        msg.data = label
        self.pub.publish(msg)

        if ch.lower() == 'q':
            self.get_logger().info('Detected "q". Shutting down node.')
            self.shutdown_node()

    def human_readable(self, s: str) -> str:
        if s == '\x03':  # Ctrl-C
            return '<CTRL-C>'
        if s == '\x1b':
            return '<ESC>'
        if s.startswith('\x1b['):
            arrows = {'A': '<UP>', 'B': '<DOWN>', 'C': '<RIGHT>', 'D': '<LEFT>'}
            return arrows.get(s[2:], f'<ESC[{s[2:]}>')
        if 32 <= ord(s[0]) <= 126:
            return s
        return f'<0x{ord(s[0]):02x}>'

    def shutdown_node(self):
        if not self._running:
            return
        self._running = False
        self.get_logger().info('Node shutting down...')
        self.restore_terminal()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        os._exit(0)  # hard exit, guaranteed


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()

    try:
        while rclpy.ok() and node._running:
            rclpy.spin_once(node, timeout_sec=0.02)
            node.poll_stdin()
    except KeyboardInterrupt:
        node.shutdown_node()
    finally:
        node.restore_terminal()
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        os._exit(0)


if __name__ == '__main__':
    main()