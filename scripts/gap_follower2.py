#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class DisparityBiggestGapFollower:
    def __init__(self):
        rospy.init_node('disparity_biggest_gap_follower')
        
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)
        
        # --- Tunable Parameters ---
        self.max_speed = 2.0             # m/s
        self.min_speed = 0.5             # m/s
        self.max_steering = 0.42         # radians (~24 deg for F1TENTH)
        
        self.car_width = 0.5             # meters
        self.safety_buffer = 0.2         # meters
        self.gap_min_width = self.car_width + self.safety_buffer

        self.disp_thresh = 0.5           # m disparity threshold for extension
        self.obstacle_thresh = 1.5       # m anything closer = obstacle
        self.forward_emergency_dist = 0.7  # m wall avoidance trigger

    def scan_callback(self, scan: LaserScan):
        ranges = np.array(scan.ranges)
        # Clean invalid readings
        ranges[np.isinf(ranges)] = scan.range_max
        ranges[np.isnan(ranges)] = 0.0

        # 1. Identify free segments
        is_free = ranges > self.obstacle_thresh
        free_segments = []
        in_free = False
        start_idx = 0
        for i in range(len(is_free)):
            if is_free[i] and not in_free:
                in_free = True
                start_idx = i
            elif (not is_free[i] or i == len(is_free)-1) and in_free:
                end_idx = i if not is_free[i] else i
                free_segments.append((start_idx, end_idx))
                in_free = False

        # 2. Pick the best gap
        best_segment = None
        best_score = -1
        for (s, e) in free_segments:
            width_angle = (e - s) * scan.angle_increment
            seg_ranges = ranges[s:e+1]
            bottleneck = np.nanmin(seg_ranges) if np.any(seg_ranges > 0) else scan.range_max

            gap_width_est = 2 * bottleneck * np.sin(width_angle/2.0)
            if gap_width_est < self.gap_min_width:
                continue

            # Center bias (favor gaps facing forward)
            gap_center_angle = scan.angle_min + ((s+e)//2) * scan.angle_increment
            center_bias = np.cos(gap_center_angle)  # near 1 when forward-facing

            score = width_angle * bottleneck * (0.5 + 0.5 * center_bias)
            if score > best_score:
                best_score = score
                best_segment = (s, e)

        if best_segment is None:
            rospy.logwarn("No safe gap found. Slowing down.")
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = self.min_speed
            drive_msg.drive.steering_angle = 0.0
            self.drive_pub.publish(drive_msg)
            return

        # 3. Apply disparity extension at edges
        s, e = best_segment
        disp = np.abs(np.diff(ranges))
        if s > 0 and disp[s-1] > self.disp_thresh:
            s = max(0, s-1)
        if e < len(disp)-1 and disp[e] > self.disp_thresh:
            e = min(len(ranges)-1, e+1)

        # 4. Lookahead bias toward turning side
        alpha = 0.7  # 0=center, 1=fully toward turning side
        center_idx = (s + e) // 2
        target_angle = scan.angle_min + center_idx * scan.angle_increment
        if target_angle > 0:  # turning left
            target_idx = int(alpha * e + (1-alpha) * s)
        else:  # turning right
            target_idx = int(alpha * s + (1-alpha) * e)

        target_angle = scan.angle_min + target_idx * scan.angle_increment

        # 5. Wall emergency override
        forward_dist = ranges[len(ranges)//2]
        if forward_dist < self.forward_emergency_dist:
            if np.nanmean(ranges[:len(ranges)//2]) > np.nanmean(ranges[len(ranges)//2:]):
                target_angle = -self.max_steering  # steer right
            else:
                target_angle = self.max_steering   # steer left

        # 6. Compute steering + dynamic speed
        steering = np.clip(target_angle, -self.max_steering, self.max_steering)
        speed = self.max_speed - (abs(steering)/self.max_steering)**2 * (self.max_speed - self.min_speed)

        # 7. Publish drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering
        drive_msg.drive.speed = speed

        rospy.loginfo(f"[GapFollower] Speed: {speed:.2f} m/s | Steering: {steering:.3f} rad")

        self.drive_pub.publish(drive_msg)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = DisparityBiggestGapFollower()
    node.run()
