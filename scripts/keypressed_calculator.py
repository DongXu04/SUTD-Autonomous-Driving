#!/usr/bin/env python3
"""
calculator_node.py
Subscribes to 'key_input' (std_msgs/String) and acts as a calculator.
Keys:
  - Digits 0–9 build numbers
  - +, -, *, / for operations
  - = to evaluate
  - c to clear/reset
"""

import rospy
from std_msgs.msg import String

expr_buffer = ""   # holds the expression being built

def key_callback(msg):
    global expr_buffer
    key = msg.data.strip()   # remove any accidental spaces

    if key.isdigit() or key in ['+', '-', '*', '/']:
        expr_buffer += key
        rospy.loginfo(f"Expression so far: {expr_buffer}")

    elif key == '=':
        if expr_buffer:
            try:
                result = eval(expr_buffer)  # ⚠️ Safe here since only controlled keys
                rospy.loginfo(f"Result: {expr_buffer} = {result}")
                expr_buffer = ""  # reset after calculation
            except Exception as e:
                rospy.logwarn(f"Error evaluating '{expr_buffer}': {e}")
                expr_buffer = ""
        else:
            rospy.loginfo("No expression to evaluate.")

    elif key.lower() == 'c':  # clear/reset
        expr_buffer = ""
        rospy.loginfo("Expression cleared.")

    else:
        rospy.logwarn(f"Ignored unknown key: {repr(key)}")

def main():
    rospy.init_node('calculator_node')
    rospy.Subscriber('key_input', String, key_callback)
    rospy.loginfo("Calculator node started. Type digits/operators on keyboard publisher. '=' to evaluate, 'c' to clear.")
    rospy.spin()

if __name__ == '__main__':
    main()
