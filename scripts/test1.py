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
        self.kp_wall = 0.12
        self.kd_wall = 0.12    # derivative gain
        self.prev_error = 0.0
        self.prev_time = rospy.get_time()
        self.wall_speed = 3.2   # base wall speed

        # ----- Gap follower params -----
        self.max_speed = 2.0
        self.min_speed = 1.4
        self.max_steering = 0.42
        self.car_width = 0.5
        self.safety_buffer = 0.2
        self.gap_min_width = self.car_width + self.safety_buffer
        self.disp_thresh = 0.5
        self.obstacle_thresh = 2.0
        self.forward_emergency_dist = 0.7

        # ----- Switching / transition params -----
        self.enter_gap_thresh = 1.75
        self.exit_gap_thresh = 2.25
        self.wall_confidence_threshold = 0.5
        self.transition_steps = 12
        self.transition_counter = 0

        # ----- Steering smoothing / safety -----
        self.prev_steering = 0.0
        self.max_steering_delta = 0.11
        self.steering_ema_alpha = 0.9  

        # ----- State -----
        self.state = 'WALL'

        # ----- Racing boost system -----
        self.current_boost = 0.0
        self.boost_rate = 0.1
        self.max_boost = 2.5
        self.decay_rate = 0.5

        rospy.loginfo("HybridFollowerSmooth ready. Starting in WALL mode.")

    def scan_callback(self, scan: LaserScan):
        ranges = np.array(scan.ranges)
        invalid = (np.isinf(ranges) | np.isnan(ranges) | (ranges == 0.0))
        ranges[invalid] = scan.range_max

        forward_idx = len(ranges) // 2
        forward_dist = ranges[forward_idx]
        wall_conf = self.compute_wall_confidence(scan, ranges)

        if self.state == 'WALL':
            if forward_dist < self.enter_gap_thresh:
                rospy.loginfo("[Switch] WALL -> GAP (obstacle ahead)")
                self.state = 'GAP'
            else:
                steer = self.compute_wall_steer(scan, ranges)
                speed = self.wall_speed

                # --- Apply boost logic in WALL mode ---
                if abs(steer) < 0.1 and forward_dist > self.forward_emergency_dist:
                    self.current_boost = min(self.max_boost, self.current_boost + self.boost_rate)
                else:
                    self.current_boost = max(0.0, self.current_boost - self.decay_rate)
                speed += self.current_boost

                self.publish_drive(steer, speed)

        elif self.state == 'GAP':
            gap_res = self.compute_gap_steer(scan, ranges)
            if gap_res is None:
                self.publish_drive(0.0, self.min_speed)
                return
            gap_steer, gap_speed = gap_res

            if (forward_dist > self.exit_gap_thresh) and (wall_conf >= self.wall_confidence_threshold):
                rospy.loginfo("[Switch] GAP -> TRANSITION (conditions met to re-acquire wall)")
                self.state = 'TRANSITION'
                self.transition_counter = 0
            else:
                self.publish_drive(gap_steer, gap_speed)

            self.current_boost = 0.0

        elif self.state == 'TRANSITION':
            gap_res = self.compute_gap_steer(scan, ranges)
            wall_steer = self.compute_wall_steer(scan, ranges)
            if gap_res is None:
                gap_steer, gap_speed = 0.0, self.min_speed
            else:
                gap_steer, gap_speed = gap_res

            alpha = float(self.transition_counter) / float(self.transition_steps)
            blended_steer = (1.0 - alpha) * gap_steer + alpha * wall_steer
            blended_speed = (1.0 - alpha) * gap_speed + alpha * self.wall_speed

            if alpha > 0.5 and abs(blended_steer) < 0.1 and forward_dist > self.forward_emergency_dist:
                self.current_boost = min(self.max_boost, self.current_boost + self.boost_rate)
            else:
                self.current_boost = max(0.0, self.current_boost - self.decay_rate)
            blended_speed += self.current_boost

            self.publish_drive(blended_steer, blended_speed)

            self.transition_counter += 1
            if self.transition_counter >= self.transition_steps:
                rospy.loginfo("[Switch] TRANSITION -> WALL (re-acquired)")
                self.state = 'WALL'

    # --- Helpers ---
    def compute_wall_confidence(self, scan, ranges):
        # check both left and right walls for "presence"
        left_min = 30.0 * np.pi / 180.0
        left_max = 120.0 * np.pi / 180.0
        right_min = -120.0 * np.pi / 180.0
        right_max = -30.0 * np.pi / 180.0

        def sector_fraction(ang_min, ang_max):
            idx_min = int((ang_min - scan.angle_min) / scan.angle_increment)
            idx_max = int((ang_max - scan.angle_min) / scan.angle_increment)
            idx_min = max(0, min(idx_min, len(ranges)-1))
            idx_max = max(0, min(idx_max, len(ranges)-1))
            if idx_max <= idx_min:
                return 0.0
            sector = ranges[idx_min:idx_max+1]
            valid = sector[(sector > 0.0) & (sector < scan.range_max)]
            if len(valid) == 0:
                return 0.0
            wall_presence_max = 3.0
            return float(np.sum(valid < wall_presence_max)) / float(len(valid))

        left_fraction = sector_fraction(left_min, left_max)
        right_fraction = sector_fraction(right_min, right_max)

        return max(left_fraction, right_fraction)

    def compute_wall_steer(self, scan, ranges):
        # sample both sides at +/- 90°
        def side_distance(angle):
            idx = int((angle - scan.angle_min) / scan.angle_increment)
            window = int(5.0 / scan.angle_increment)
            window = max(1, min(window, 5))
            i0 = max(0, idx - window)
            i1 = min(len(ranges)-1, idx + window)
            sector = ranges[i0:i1+1]
            valid = sector[(sector > 0.0) & (sector < scan.range_max)]
            return float(np.median(valid)) if len(valid) > 0 else scan.range_max

        right_dist = side_distance(-90.0 * np.pi / 180.0)
        left_dist = side_distance(90.0 * np.pi / 180.0)

        # error = difference (we want left == right)
        error = left_dist - right_dist

        now = rospy.get_time()
        dt = now - self.prev_time if self.prev_time is not None else 0.0
        de = (error - self.prev_error) / dt if dt > 1e-6 else 0.0

        steering = self.kp_wall * error + self.kd_wall * de
        steering = np.clip(steering, -self.max_steering, self.max_steering)

        self.prev_error = error
        self.prev_time = now

        return steering

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

        best_segment, best_score = None, -1.0
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
                best_score, best_segment = score, (s, e)

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
        target_idx = int(alpha * e + (1.0 - alpha) * s) if target_angle > 0 else int(alpha * s + (1.0 - alpha) * e)
        target_idx = max(0, min(target_idx, len(ranges)-1))
        target_angle = scan.angle_min + target_idx * scan.angle_increment

        forward_dist = ranges[len(ranges)//2]
        if forward_dist < self.forward_emergency_dist:
            left_mean = np.nanmean(ranges[len(ranges)//2:])
            right_mean = np.nanmean(ranges[:len(ranges)//2])
            target_angle = -self.max_steering if left_mean > right_mean else self.max_steering

        steering = np.clip(target_angle, -self.max_steering, self.max_steering)
        speed = self.max_speed - (abs(steering) / self.max_steering) ** 2 * (self.max_speed - self.min_speed)
        return steering, speed

    def publish_drive(self, steering, speed):
        delta = steering - self.prev_steering
        if abs(delta) > self.max_steering_delta:
            steering = self.prev_steering + np.sign(delta) * self.max_steering_delta
        steering = self.steering_ema_alpha * steering + (1.0 - self.steering_ema_alpha) * self.prev_steering
        steering = np.clip(steering, -self.max_steering, self.max_steering)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = float(np.clip(speed, self.min_speed, self.wall_speed + self.max_boost))
        drive_msg.drive.steering_angle = float(steering)
        self.drive_pub.publish(drive_msg)
        self.prev_steering = steering

if __name__ == '__main__':
    node = HybridFollowerSmooth()
    rospy.spin()
