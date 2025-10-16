#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

pub = None

def callback(scan):
    # Take the front sector (e.g., +/- 10° around front)
    num_ranges = len(scan.ranges)
    front_ranges = scan.ranges[num_ranges//2 - 10 : num_ranges//2 + 10]

    # Minimum distance in front
    min_front = min(front_ranges)

    msg = AckermannDriveStamped()

    if min_front < 1.0:  # obstacle closer than 1m
        # Stop and steer left
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.5  # turn left (~30 deg)
        rospy.loginfo("Obstacle detected! Turning left.")
    else:
        # Drive forward
        msg.drive.speed = 1.0
        msg.drive.steering_angle = 0.0
        rospy.loginfo("Path clear. Driving forward.")

    pub.publish(msg)

def avoider():
    global pub
    rospy.init_node('obstacle_avoider', anonymous=True)
    pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        avoider()
    except rospy.ROSInterruptException:
        pass
