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
        
        # --- tunable parameters ---
        self.max_speed = 2.0            # m/s
        self.min_speed = 0.5            # m/s
        self.max_steering = 0.35        # radians
        
        self.car_width = 0.5            # meters, width of the car
        self.safety_buffer = 0.2        # extra margin so you're conservative
        self.gap_min_width = self.car_width + self.safety_buffer  # smallest gap you consider
        
    def scan_callback(self, scan: LaserScan):
        ranges = np.array(scan.ranges)
        # clean up bad readings
        valid = np.logical_and(ranges > scan.range_min, ranges < scan.range_max)
        # set invalid to nan so they don't get picked
        ranges[~valid] = np.nan
        
        # 1. Disparity: detect edges where there's a big jump
        disp = np.abs(np.diff(ranges))
        
        # We'll mark indices where the disparity is high enough that one side is an obstacle edge
        # But full gap detection is needed: between two obstacles, is there enough width?
        
        # 2. Identify obstacle points (too close), to treat as boundaries
        obstacle_thresh = 1.5  # meters — closer than this is obstacle (you can tune)
        is_obstacle = ranges < obstacle_thresh
        
        # 3. Find contiguous segments of “free space”
        # free = ranges sufficiently far (or nan)
        free_thresh = obstacle_thresh
        is_free = np.logical_or(ranges > free_thresh, np.isnan(ranges))
        
        # Find start and end indices of free segments
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
        
        # 4. From all free segments, pick the one with largest angular width *range approx
        best_segment = None
        best_score = -1
        for (s, e) in free_segments:
            # Compute width: how many beams * angle increment
            width_angle = (e - s) * scan.angle_increment
            # Also factor in how far away obstacles are in that segment (so longer range better)
            # We'll take the minimal distance in that segment as bottleneck
            seg_ranges = ranges[s:e+1]
            # ignore nan in computing min
            if np.all(np.isnan(seg_ranges)):
                # all nan => treat as very far
                bottleneck = scan.range_max
            else:
                bottleneck = np.nanmin(seg_ranges)
            
            # Check if physical gap > car width; calculate approximate gap width using two endpoints
            # gap_width_est = 2 * bottleneck * sin(width_angle/2)
            gap_width_est = 2 * bottleneck * np.sin(width_angle / 2.0)
            if gap_width_est < self.gap_min_width:
                continue
            
            # Score by combining width_angle and bottleneck
            score = width_angle * bottleneck
            if score > best_score:
                best_score = score
                best_segment = (s, e, bottleneck, width_angle)
        
        if best_segment is None:
            # no safe gap found -> slow down, maybe stop or just pick best available direction
            rospy.logwarn("No safe gap found. Slowing down.")
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = self.min_speed
            drive_msg.drive.steering_angle = 0.0
            self.drive_pub.publish(drive_msg)
            return
        
        # 5. Disparity Extension at edges of the selected gap
        s, e, bottleneck, width_angle = best_segment
        
        # find edges in disparity around s and e
        # For simplicity, extend s backward and e forward by some indices based on disparity
        # let's find disparity > some threshold near the edges
        disp_thresh = 0.5  # meters — when disparity jump is big
        # Check at s-1 (if valid)
        extend_left = s
        if s > 0 and disp[s-1] > disp_thresh:
            # extend by 1 beam
            extend_left = max(0, s-1)
        # Check at e (disp between e and e+1)
        extend_right = e
        if e < len(disp)-1 and disp[e] > disp_thresh:
            extend_right = min(len(ranges)-1, e+1)
        
        # New target index = middle of the extended gap
        target_idx = (extend_left + extend_right) // 2
        target_angle = scan.angle_min + target_idx * scan.angle_increment
        
        # 6. Compute steering, speed based on how sharp the angle is
        steering = np.clip(target_angle, -self.max_steering, self.max_steering)
        # speed reduces when steering is large
        speed = self.max_speed - (abs(steering) / self.max_steering) * (self.max_speed - self.min_speed)
        
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering
        self.drive_pub.publish(drive_msg)
    
    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = DisparityBiggestGapFollower()
    node.run()
