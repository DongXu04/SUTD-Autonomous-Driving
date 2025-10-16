#!/usr/bin/env python3
"""
calculator_node_ros2.py
Subscribes to 'key_input' (std_msgs/String) and acts as a calculator.
Keys:
  - Digits 0–9 build numbers
  - +, -, *, / for operations
  - = to evaluate
  - c to clear/reset
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CalculatorNode(Node):
    def __init__(self):
        super().__init__('calculator_node')
        self.expr_buffer = ""  # holds the expression being built
        self.subscription = self.create_subscription(
            String,
            'key_input',
            self.key_callback,
            10
        )
        self.get_logger().info(
            "Calculator node started. Type digits/operators on keyboard publisher. '=' to evaluate, 'c' to clear."
        )

    def key_callback(self, msg):
        key = msg.data.strip()  # remove accidental spaces

        if key.isdigit() or key in ['+', '-', '*', '/']:
            self.expr_buffer += key
            self.get_logger().info(f"Expression so far: {self.expr_buffer}")

        elif key == '=':
            if self.expr_buffer:
                try:
                    # ⚠️ Safe here since only controlled keys are used
                    result = eval(self.expr_buffer)
                    self.get_logger().info(f"Result: {self.expr_buffer} = {result}")
                    self.expr_buffer = ""  # reset after calculation
                except Exception as e:
                    self.get_logger().warn(f"Error evaluating '{self.expr_buffer}': {e}")
                    self.expr_buffer = ""
            else:
                self.get_logger().info("No expression to evaluate.")

        elif key.lower() == 'c':  # clear/reset
            self.expr_buffer = ""
            self.get_logger().info("Expression cleared.")

        else:
            self.get_logger().warn(f"Ignored unknown key: {repr(key)}")


def main(args=None):
    rclpy.init(args=args)
    node = CalculatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
