#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class SmarterGapFollower:
    def __init__(self):
        rospy.init_node("smarter_gap_follower")

        # Publisher to control the car
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=10)

        # Subscriber to LiDAR
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # Parameters
        self.speed = 1.5
        self.max_steer = 0.6  # rad, max steering angle
        self.bubble_radius = 80  # ignore around closest obstacle (in indices)
        rospy.loginfo("Smarter Gap Follower node started")
        rospy.spin()

    def scan_callback(self, scan: LaserScan):
        ranges = np.array(scan.ranges)
        ranges = np.nan_to_num(ranges, nan=scan.range_max, posinf=scan.range_max, neginf=0.0)

        # Step 1: Mask bubble around the closest obstacle
        closest_idx = np.argmin(ranges)
        min_dist = ranges[closest_idx]
        start_idx = max(0, closest_idx - self.bubble_radius)
        end_idx = min(len(ranges) - 1, closest_idx + self.bubble_radius)
        ranges[start_idx:end_idx] = 0.0

        # Step 2: Find the largest gap
        gaps = []
        gap_start = None
        for i, r in enumerate(ranges):
            if r > 0 and gap_start is None:
                gap_start = i
            elif r == 0 and gap_start is not None:
                gaps.append((gap_start, i - 1))
                gap_start = None
        if gap_start is not None:
            gaps.append((gap_start, len(ranges) - 1))

        if not gaps:
            rospy.logwarn("No gap found!")
            return

        largest_gap = max(gaps, key=lambda g: g[1] - g[0])
        gap_start, gap_end = largest_gap

        # Step 3: Select the furthest point inside the gap (bias toward forward center)
        gap_ranges = ranges[gap_start:gap_end + 1]
        furthest_idx_in_gap = np.argmax(gap_ranges) + gap_start

        # Bias towards center (car forward direction = middle index of LiDAR)
        center_idx = len(ranges) // 2
        weighted_idx = int((furthest_idx_in_gap + center_idx) / 2)

        # Step 4: Convert chosen index to angle
        target_angle = scan.angle_min + weighted_idx * scan.angle_increment
        steer = np.clip(target_angle, -self.max_steer, self.max_steer)

        # Step 5: Publish drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.speed
        drive_msg.drive.steering_angle = steer
        self.drive_pub.publish(drive_msg)

        rospy.loginfo("Steering=%.2f rad | Speed=%.2f | Gap=(%d,%d) | FurthestIdx=%d | WeightedIdx=%d"
                      % (steer, self.speed, gap_start, gap_end, furthest_idx_in_gap, weighted_idx))

if __name__ == "__main__":
    try:
        SmarterGapFollower()
    except rospy.ROSInterruptException:
        pass
