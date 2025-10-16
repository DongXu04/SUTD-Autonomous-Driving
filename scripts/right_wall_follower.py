#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class WallFollower:
    def __init__(self):
        rospy.init_node('wall_follower', anonymous=True)

        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.desired_distance = 0.8   # target distance to right wall (meters)
        self.kp = 1.2                 # proportional gain
        self.speed = 1.5              # forward speed

    def scan_callback(self, scan):
        # Index for ~90 degrees to the right (depending on scan resolution)
        right_angle = -90 * np.pi/180
        right_index = int((right_angle - scan.angle_min) / scan.angle_increment)

        # Clamp index to scan range
        right_index = max(0, min(right_index, len(scan.ranges)-1))
        right_dist = scan.ranges[right_index]

        # Error = current distance - desired distance
        error = self.desired_distance - right_dist

        # Proportional controller for steering
        steering_angle = self.kp * error

        # Publish drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.speed
        drive_msg.drive.steering_angle = steering_angle
        self.drive_pub.publish(drive_msg)

if __name__ == '__main__':
    WallFollower()
    rospy.spin()
