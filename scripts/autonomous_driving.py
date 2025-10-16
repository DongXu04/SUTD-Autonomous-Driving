#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class HybridFollower:
    def __init__(self):
        rospy.init_node('hybrid_follower', anonymous=True)

        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # --- Wall Follower Params ---
        self.desired_distance = 0.8   # meters to right wall
        self.kp = 1.2
        self.wall_speed = 1.5

        # --- Gap Follower Params ---
        self.max_speed = 2.0
        self.min_speed = 0.5
        self.max_steering = 0.42
        self.car_width = 0.5
        self.safety_buffer = 0.2
        self.gap_min_width = self.car_width + self.safety_buffer
        self.disp_thresh = 0.5
        self.obstacle_thresh = 1.5
        self.forward_emergency_dist = 0.7

        # --- Switching Params ---
        self.switch_thresh = 1.5  # if obstacle ahead closer than this, use gap mode

    def scan_callback(self, scan: LaserScan):
        ranges = np.array(scan.ranges)
        ranges[np.isinf(ranges)] = scan.range_max
        ranges[np.isnan(ranges)] = 0.0

        forward_dist = ranges[len(ranges)//2]

        if forward_dist < self.switch_thresh:
            # --- GAP FOLLOWER MODE ---
            self.run_gap_follower(scan, ranges)
        else:
            # --- WALL FOLLOWER MODE ---
            self.run_wall_follower(scan, ranges)

    # ------------------- WALL FOLLOWER -------------------
    def run_wall_follower(self, scan, ranges):
        right_angle = -90 * np.pi/180
        right_index = int((right_angle - scan.angle_min) / scan.angle_increment)
        right_index = max(0, min(right_index, len(ranges)-1))
        right_dist = ranges[right_index]

        error = self.desired_distance - right_dist
        steering_angle = self.kp * error

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.wall_speed
        drive_msg.drive.steering_angle = steering_angle
        rospy.loginfo(f"[WallFollower] Speed: {self.wall_speed:.2f} | Steering: {steering_angle:.3f}")
        self.drive_pub.publish(drive_msg)

    # ------------------- GAP FOLLOWER -------------------
    def run_gap_follower(self, scan, ranges):
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

        best_segment, best_score = None, -1
        for (s, e) in free_segments:
            width_angle = (e - s) * scan.angle_increment
            seg_ranges = ranges[s:e+1]
            bottleneck = np.nanmin(seg_ranges) if np.any(seg_ranges > 0) else scan.range_max
            gap_width_est = 2 * bottleneck * np.sin(width_angle/2.0)
            if gap_width_est < self.gap_min_width:
                continue
            gap_center_angle = scan.angle_min + ((s+e)//2) * scan.angle_increment
            center_bias = np.cos(gap_center_angle)
            score = width_angle * bottleneck * (0.5 + 0.5 * center_bias)
            if score > best_score:
                best_score = score
                best_segment = (s, e)

        if best_segment is None:
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = self.min_speed
            drive_msg.drive.steering_angle = 0.0
            rospy.logwarn("[GapFollower] No safe gap found. Slowing down.")
            self.drive_pub.publish(drive_msg)
            return

        s, e = best_segment
        disp = np.abs(np.diff(ranges))
        if s > 0 and disp[s-1] > self.disp_thresh:
            s = max(0, s-1)
        if e < len(disp)-1 and disp[e] > self.disp_thresh:
            e = min(len(ranges)-1, e+1)

        alpha = 0.7
        center_idx = (s + e) // 2
        target_angle = scan.angle_min + center_idx * scan.angle_increment
        if target_angle > 0:
            target_idx = int(alpha * e + (1-alpha) * s)
        else:
            target_idx = int(alpha * s + (1-alpha) * e)
        target_angle = scan.angle_min + target_idx * scan.angle_increment

        forward_dist = ranges[len(ranges)//2]
        if forward_dist < self.forward_emergency_dist:
            if np.nanmean(ranges[:len(ranges)//2]) > np.nanmean(ranges[len(ranges)//2:]):
                target_angle = -self.max_steering
            else:
                target_angle = self.max_steering

        steering = np.clip(target_angle, -self.max_steering, self.max_steering)
        speed = self.max_speed - (abs(steering)/self.max_steering)**2 * (self.max_speed - self.min_speed)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering
        drive_msg.drive.speed = speed
        rospy.loginfo(f"[GapFollower] Speed: {speed:.2f} | Steering: {steering:.3f}")
        self.drive_pub.publish(drive_msg)


if __name__ == '__main__':
    node = HybridFollower()
    rospy.spin()
