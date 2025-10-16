#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class HybridFollowerSmooth:
    def __init__(self):
        rospy.init_node('hybrid_follower_smooth', anonymous=True)

        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # ----- Wall follower params -----
        self.desired_distance = 0.8   # desired distance to right wall (m)   [Berlin: 1.2 Levine: 0.8]
        self.kp_wall = 1.2
        self.wall_speed = 5.0

        # ----- Gap follower params (same idea as before) -----
        self.max_speed = 2.0
        self.min_speed = 0.5
        self.max_steering = 0.42
        self.car_width = 0.5
        self.safety_buffer = 0.2
        self.gap_min_width = self.car_width + self.safety_buffer
        self.disp_thresh = 0.5
        self.obstacle_thresh = 2.0
        self.forward_emergency_dist = 0.7

        # ----- Switching / transition params -----
        self.enter_gap_thresh = 1.75   # if forward < enter => go to GAP immediately
        self.exit_gap_thresh = 2.25    # need forward > exit AND wall_confidence to go back
        self.wall_confidence_threshold = 0.5  # fraction of valid right-sector readings required
        self.transition_steps = 12    # number of scans over which to blend gap->wall
        self.transition_counter = 0

        # ----- Steering smoothing / safety -----
        self.prev_steering = 0.0
        self.max_steering_delta = 0.08  # max change in steering per callback (radians)
        self.steering_ema_alpha = 0.7    # small smoothing of published steering

        # ----- State -----
        self.state = 'WALL'  # other states: 'GAP', 'TRANSITION'

        rospy.loginfo("HybridFollowerSmooth ready. Starting in WALL mode.")

    # ----------------------- Main callback -----------------------
    def scan_callback(self, scan: LaserScan):
        ranges = np.array(scan.ranges)
        # Replace invalid readings with range_max (treat as far)
        invalid = (np.isinf(ranges) | np.isnan(ranges) | (ranges == 0.0))
        ranges[invalid] = scan.range_max

        forward_idx = len(ranges) // 2
        forward_dist = ranges[forward_idx]

        # compute right-side wall confidence
        wall_conf = self.compute_wall_confidence(scan, ranges)

        if self.state == 'WALL':
            if forward_dist < self.enter_gap_thresh:
                rospy.loginfo("[Switch] WALL -> GAP (obstacle ahead)")
                self.state = 'GAP'
            else:
                steer = self.compute_wall_steer(scan, ranges)
                speed = self.wall_speed
                self.publish_drive(steer, speed)

        elif self.state == 'GAP':
            gap_res = self.compute_gap_steer(scan, ranges)
            if gap_res is None:
                # no gap: slow and stop-ish
                self.publish_drive(0.0, self.min_speed)
                return
            gap_steer, gap_speed = gap_res

            # Condition to begin transition back to wall-follow
            if (forward_dist > self.exit_gap_thresh) and (wall_conf >= self.wall_confidence_threshold):
                rospy.loginfo("[Switch] GAP -> TRANSITION (conditions met to re-acquire wall)")
                self.state = 'TRANSITION'
                self.transition_counter = 0
            else:
                self.publish_drive(gap_steer, gap_speed)

        elif self.state == 'TRANSITION':
            gap_res = self.compute_gap_steer(scan, ranges)
            wall_steer = self.compute_wall_steer(scan, ranges)
            if gap_res is None:
                gap_steer, gap_speed = 0.0, self.min_speed
            else:
                gap_steer, gap_speed = gap_res

            alpha = float(self.transition_counter) / float(self.transition_steps)
            # alpha = 0 -> fully gap, alpha = 1 -> fully wall
            blended_steer = (1.0 - alpha) * gap_steer + alpha * wall_steer
            blended_speed = (1.0 - alpha) * gap_speed + alpha * self.wall_speed

            self.publish_drive(blended_steer, blended_speed)

            self.transition_counter += 1
            if self.transition_counter >= self.transition_steps:
                rospy.loginfo("[Switch] TRANSITION -> WALL (re-acquired)")
                self.state = 'WALL'

    # ----------------------- Helper: wall confidence -----------------------
    def compute_wall_confidence(self, scan, ranges):
        # Look at a right-side sector (e.g. -120 deg to -30 deg) for wall continuity
        right_min = -120.0 * np.pi / 180.0
        right_max = -30.0 * np.pi / 180.0
        idx_min = int((right_min - scan.angle_min) / scan.angle_increment)
        idx_max = int((right_max - scan.angle_min) / scan.angle_increment)
        idx_min = max(0, min(idx_min, len(ranges)-1))
        idx_max = max(0, min(idx_max, len(ranges)-1))
        if idx_max <= idx_min:
            return 0.0
        sector = ranges[idx_min:idx_max+1]
        valid = sector[(sector > 0.0) & (sector < scan.range_max)]
        if len(valid) == 0:
            return 0.0
        # fraction of readings that are "near enough" (indicating a wall)
        wall_presence_max = 3.0  # consider readings < 3.0m as possible wall returns
        fraction = float(np.sum(valid < wall_presence_max)) / float(len(valid))
        return fraction  # between 0 and 1

    # ----------------------- Helper: wall steering -----------------------
    def compute_wall_steer(self, scan, ranges):
        # Use a small window centered around -90 degrees for robust right-dist estimate
        right_angle = -90.0 * np.pi / 180.0
        center_idx = int((right_angle - scan.angle_min) / scan.angle_increment)
        window = int(5.0 / (scan.angle_increment))  # ~5 rad? check units - keep small
        # ensure sanity
        window = max(1, min(window, 5))
        i0 = max(0, center_idx - window)
        i1 = min(len(ranges)-1, center_idx + window)
        sector = ranges[i0:i1+1]
        valid = sector[(sector > 0.0) & (sector < scan.range_max)]
        if len(valid) == 0:
            # fallback
            measured = scan.range_max
        else:
            measured = float(np.median(valid))  # median helps with outliers

        error = self.desired_distance - measured
        steering = self.kp_wall * error
        steering = np.clip(steering, -self.max_steering, self.max_steering)
        return steering

    # ----------------------- Helper: gap steering (refactored) -----------------------
    def compute_gap_steer(self, scan, ranges):
        is_free = ranges > self.obstacle_thresh
        free_segments = []
        in_free = False
        start_idx = 0
        N = len(is_free)

        for i in range(N):
            if is_free[i] and not in_free:
                in_free = True
                start_idx = i
            elif (not is_free[i] or i == N-1) and in_free:
                end_idx = i if not is_free[i] else i
                free_segments.append((start_idx, end_idx))
                in_free = False

        best_segment = None
        best_score = -1.0
        for (s, e) in free_segments:
            width_angle = (e - s) * scan.angle_increment
            seg_ranges = ranges[s:e+1]
            bottleneck = np.nanmin(seg_ranges) if np.any(seg_ranges > 0) else scan.range_max
            gap_width_est = 2.0 * bottleneck * np.sin(width_angle / 2.0)
            if gap_width_est < self.gap_min_width:
                continue

            gap_center_angle = scan.angle_min + ((s + e) // 2) * scan.angle_increment
            center_bias = np.cos(gap_center_angle)
            score = width_angle * bottleneck * (0.5 + 0.5 * center_bias)
            if score > best_score:
                best_score = score
                best_segment = (s, e)

        if best_segment is None:
            return None

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
            target_idx = int(alpha * e + (1.0 - alpha) * s)
        else:
            target_idx = int(alpha * s + (1.0 - alpha) * e)

        target_idx = max(0, min(target_idx, len(ranges)-1))
        target_angle = scan.angle_min + target_idx * scan.angle_increment

        forward_dist = ranges[len(ranges)//2]
        if forward_dist < self.forward_emergency_dist:
            # emergency lateral decision based on side means
            left_mean = np.nanmean(ranges[len(ranges)//2:])
            right_mean = np.nanmean(ranges[:len(ranges)//2])
            if left_mean > right_mean:
                target_angle = -self.max_steering
            else:
                target_angle = self.max_steering

        steering = np.clip(target_angle, -self.max_steering, self.max_steering)
        speed = self.max_speed - (abs(steering) / self.max_steering) ** 2 * (self.max_speed - self.min_speed)
        return steering, speed

    # ----------------------- Publish with smoothing & rate limiting -----------------------
    def publish_drive(self, steering, speed):
        # rate-limit steering change
        delta = steering - self.prev_steering
        if abs(delta) > self.max_steering_delta:
            steering = self.prev_steering + np.sign(delta) * self.max_steering_delta

        # small EMA to avoid jitter
        steering = self.steering_ema_alpha * steering + (1.0 - self.steering_ema_alpha) * self.prev_steering

        # clamp
        steering = np.clip(steering, -self.max_steering, self.max_steering)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = float(np.clip(speed, self.min_speed, self.max_speed))
        drive_msg.drive.steering_angle = float(steering)
        self.drive_pub.publish(drive_msg)

        self.prev_steering = steering

if __name__ == '__main__':
    node = HybridFollowerSmooth()
    rospy.spin()
