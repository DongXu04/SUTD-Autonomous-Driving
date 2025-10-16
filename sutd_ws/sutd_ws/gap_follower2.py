#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class DisparityBiggestGapFollower(Node):
    def __init__(self):
        super().__init__('disparity_biggest_gap_follower')
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)
        
        # --- tunable parameters ---
        self.max_speed = 2.0            # m/s
        self.min_speed = 0.5            # m/s
        self.max_steering = 0.45        # radians
        
        self.car_width = 0.5            # meters, width of the car
        self.safety_buffer = 0.2        # extra margin
        self.gap_min_width = self.car_width + self.safety_buffer

    def scan_callback(self, scan: LaserScan):
        ranges = np.array(scan.ranges)
        valid = np.logical_and(ranges > scan.range_min, ranges < scan.range_max)
        ranges[~valid] = np.nan
        
        disp = np.abs(np.diff(ranges))
        
        obstacle_thresh = 1.5
        is_obstacle = ranges < obstacle_thresh
        
        free_thresh = obstacle_thresh
        is_free = np.logical_or(ranges > free_thresh, np.isnan(ranges))
        
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
        
        best_segment = None
        best_score = -1
        for (s, e) in free_segments:
            width_angle = (e - s) * scan.angle_increment
            seg_ranges = ranges[s:e+1]
            if np.all(np.isnan(seg_ranges)):
                bottleneck = scan.range_max
            else:
                bottleneck = np.nanmin(seg_ranges)
            
            gap_width_est = 2 * bottleneck * np.sin(width_angle / 2.0)
            if gap_width_est < self.gap_min_width:
                continue
            
            score = width_angle * bottleneck
            if score > best_score:
                best_score = score
                best_segment = (s, e, bottleneck, width_angle)
        
        if best_segment is None:
            self.get_logger().warn("No safe gap found. Slowing down.")
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = self.min_speed
            drive_msg.drive.steering_angle = 0.0
            self.drive_pub.publish(drive_msg)
            return
        
        s, e, bottleneck, width_angle = best_segment
        
        disp_thresh = 0.5
        extend_left = s
        if s > 0 and disp[s-1] > disp_thresh:
            extend_left = max(0, s-1)
        extend_right = e
        if e < len(disp)-1 and disp[e] > disp_thresh:
            extend_right = min(len(ranges)-1, e+1)
        
        target_idx = (extend_left + extend_right) // 2
        target_angle = scan.angle_min + target_idx * scan.angle_increment
        
        steering = np.clip(target_angle, -self.max_steering, self.max_steering)
        speed = self.max_speed - (abs(steering) / self.max_steering) * (self.max_speed - self.min_speed)
        
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = float(speed)
        drive_msg.drive.steering_angle = float(steering)
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DisparityBiggestGapFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
