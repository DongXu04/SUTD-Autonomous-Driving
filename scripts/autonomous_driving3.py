#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class HybridRacer:
    def __init__(self):
        rospy.init_node("hybrid_racer", anonymous=True)

        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # --- Wall follower parameters ---
        self.desired_distance = 0.8
        self.kp = 1.0

        # --- Gap follower parameters ---
        self.disp_thresh = 0.5
        self.obstacle_thresh = 1.5
        # earlier obstacle detection
        self.forward_emergency_dist = 1.3   # was 0.7

        # --- Car + racing parameters ---
        self.car_width = 0.5
        self.safety_buffer = 0.2
        self.gap_min_width = self.car_width + self.safety_buffer
        self.max_speed = 3.5
        self.min_speed = 0.8
        self.max_steering = 0.42  # radians

        # --- Blending & stability ---
        self.alpha = 1.0          # 1 = wall, 0 = gap
        self.alpha_decay = 0.002  # slower return to wall (was 0.01)
        self.alpha_rise = 0.4     # faster switch to gap (was 0.1)
        self.prev_steer = 0.0

    def scan_callback(self, scan: LaserScan):
        ranges = np.array(scan.ranges)
        ranges[np.isinf(ranges)] = scan.range_max
        ranges[np.isnan(ranges)] = 0.0

        # --- Wall follower steering ---
        right_angle = -90 * np.pi / 180
        right_index = int((right_angle - scan.angle_min) / scan.angle_increment)
        right_index = max(0, min(right_index, len(ranges) - 1))
        right_dist = ranges[right_index]
        wall_error = self.desired_distance - right_dist
        steer_wall = self.kp * wall_error

        # --- Gap follower steering ---
        steer_gap = self.find_gap_steering(scan, ranges)

        # --- Mode switching logic ---
        forward_dist = ranges[len(ranges)//2]
        if forward_dist < self.forward_emergency_dist:
            # force gap mode quickly
            self.alpha = max(0.0, self.alpha - self.alpha_rise)
        else:
            # drift slowly back to wall mode
            self.alpha = min(1.0, self.alpha + self.alpha_decay)

        # --- Blend steering ---
        steering = (self.alpha * steer_wall) + ((1 - self.alpha) * steer_gap)

        # --- Low-pass filter steering (smoothing) ---
        steering = 0.7 * self.prev_steer + 0.3 * steering
        self.prev_steer = steering

        # --- Steering cap based on speed ---
        current_speed = self.dynamic_speed(steering)
        steer_limit = self.max_steering * (self.max_speed / max(current_speed, 0.1))
        steering = np.clip(steering, -steer_limit, steer_limit)

        # --- Publish ---
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = current_speed
        drive_msg.drive.steering_angle = steering
        self.drive_pub.publish(drive_msg)

        rospy.loginfo(f"[HybridRacer] speed={current_speed:.2f}, steer={steering:.3f}, alpha={self.alpha:.2f}")

    def find_gap_steering(self, scan: LaserScan, ranges: np.ndarray):
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
                best_score, best_segment = score, (s, e)

        if best_segment is None:
            return 0.0

        s, e = best_segment
        center_idx = (s + e) // 2
        target_angle = scan.angle_min + center_idx * scan.angle_increment
        return np.clip(target_angle, -self.max_steering, self.max_steering)

    def dynamic_speed(self, steering):
        # Corner = slow, straight = fast
        speed = self.max_speed - (abs(steering)/self.max_steering)**2 * (self.max_speed - self.min_speed)
        return np.clip(speed, self.min_speed, self.max_speed)

if __name__ == "__main__":
    node = HybridRacer()
    rospy.spin()
