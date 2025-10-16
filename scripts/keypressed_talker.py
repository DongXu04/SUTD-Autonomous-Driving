#!/usr/bin/env python3
"""
keyboard_publisher.py
Publishes single-key presses to topic 'key_input' as std_msgs/String.
Press w/a/s/d for directions, q to quit.
"""
import sys
import select
import termios
import tty
import rospy
from std_msgs.msg import String

def getKey(timeout=0.1):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            c = sys.stdin.read(1)
        else:
            c = ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return c

def main():
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('keyboard_publisher')
    pub = rospy.Publisher('key_input', String, queue_size=1)
    rate = rospy.Rate(10)  # 10 Hz loop

    print("Keyboard publisher started. Press w/a/s/d to move, q to quit.")
    try:
        while not rospy.is_shutdown():
            key = getKey(0.1)
            if key:
                if key == 'q':
                    print("Quitting.")
                    break
                msg = String()
                msg.data = key
                pub.publish(msg)
                # optional feedback:
                rospy.loginfo(f"Published key: {repr(key)}")
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
