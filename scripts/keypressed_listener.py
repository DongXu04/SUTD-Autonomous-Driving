#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(msg):
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

def main():
    rospy.init_node('keyboard_listener')
    rospy.Subscriber('key_input', String, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
