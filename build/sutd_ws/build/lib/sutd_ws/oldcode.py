#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class HybridFollower(Node):
    def __init__(self):
        super().__init__('hybrid_follower')

        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # --- Wall Follower Params ---
        self.desired_distance = 0.8  # meters to right wall
        self.kp = 0.8
        self.wall_speed = 4.5

        # --- Gap Follower Params ---
        self.max_speed = 5.0
        self.min_speed = 1.0
        self.max_steering = 0.42
        self.car_width = 0.5
        self.safety_buffer = 0.2
        self.gap_min_width = self.car_width + self.safety_buffer
        self.disp_thresh = 0.5
        self.obstacle_thresh = 1.5
        self.forward_emergency_dist = 1.3  # ↑ safer lookahead for obstacles

        # --- Switching Params (hybrid smoothing) ---
        self.switch_thresh = 1.8   # ↑ trigger gap mode earlier
        self.alpha_rise = 0.4      # ↑ faster rise
        self.alpha_decay = 0.002   # ↓ slower decay
        self.gap_confidence = 0.0  # internal blending value

    def scan_callback(self, scan: LaserScan):
        ranges = np.array(scan.ranges)
        ranges[np.isinf(ranges)] = scan.range_max
        ranges[np.isnan(ranges)] = 0.0

        forward_dist = ranges[len(ranges) // 2]

        # Blend wall/gap confidence dynamically
        target_conf = 1.0 if forward_dist < self.switch_thresh else 0.0
        alpha = self.alpha_rise if target_conf > self.gap_confidence else self.alpha_decay
        self.gap_confidence = (1 - alpha) * self.gap_confidence + alpha * target_conf

        if self.gap_confidence > 0.5:
            self.run_gap_follower(scan, ranges)
        else:
            self.run_wall_follower(scan, ranges)

    # ------------------- WALL FOLLOWER -------------------
    def run_wall_follower(self, scan, ranges):
        right_angle = -90 * np.pi / 180
        right_index = int((right_angle - scan.angle_min) / scan.angle_increment)
        right_index = max(0, min(right_index, len(ranges) - 1))
        right_dist = ranges[right_index]

        error = self.desired_distance - right_dist
        steering_angle = self.kp * error

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.wall_speed
        drive_msg.drive.steering_angle = steering_angle
        self.get_logger().info(f"[WallFollower] Speed: {self.wall_speed:.2f} | Steering: {steering_angle:.3f}")
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
            elif (not is_free[i] or i == len(is_free) - 1) and in_free:
                end_idx = i if not is_free[i] else i
                free_segments.append((start_idx, end_idx))
                in_free = False

        best_segment, best_score = None, -1
        for (s, e) in free_segments:
            width_angle = (e - s) * scan.angle_increment
            seg_ranges = ranges[s:e + 1]
            bottleneck = np.nanmin(seg_ranges) if np.any(seg_ranges > 0) else scan.range_max
            gap_width_est = 2 * bottleneck * np.sin(width_angle / 2.0)
            if gap_width_est < self.gap_min_width:
                continue
            gap_center_angle = scan.angle_min + ((s + e) // 2) * scan.angle_increment
            center_bias = np.cos(gap_center_angle)
            score = width_angle * bottleneck * (0.5 + 0.5 * center_bias)
            if score > best_score:
                best_score = score
                best_segment = (s, e)

        if best_segment is None:
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = self.min_speed
            drive_msg.drive.steering_angle = 0.0
            self.get_logger().warn("[GapFollower] No safe gap found. Slowing down.")
            self.drive_pub.publish(drive_msg)
            return

        s, e = best_segment
        disp = np.abs(np.diff(ranges))
        if s > 0 and disp[s - 1] > self.disp_thresh:
            s = max(0, s - 1)
        if e < len(disp) - 1 and disp[e] > self.disp_thresh:
            e = min(len(ranges) - 1, e + 1)

        alpha = 0.7
        center_idx = (s + e) // 2
        target_angle = scan.angle_min + center_idx * scan.angle_increment
        if target_angle > 0:
            target_idx = int(alpha * e + (1 - alpha) * s)
        else:
            target_idx = int(alpha * s + (1 - alpha) * e)
        target_angle = scan.angle_min + target_idx * scan.angle_increment

        forward_dist = ranges[len(ranges) // 2]
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
        self.get_logger().info(f"[GapFollower] Speed: {speed:.2f} | Steering: {steering:.3f}")
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HybridFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
