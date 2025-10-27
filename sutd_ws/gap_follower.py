#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import sys, termios, tty, select, threading

class DisparityBiggestGapFollower(Node):
    def __init__(self):
        super().__init__('disparity_biggest_gap_follower')
        
        # --- ROS2 interfaces ---
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)
        
        # --- Tunable parameters ---
        self.max_speed = 1.0        
        self.min_speed = 0.25
        self.max_steering = 0.5

        self.car_width = 0.5            #safety buffer for the car's width
        self.safety_buffer = 0.2        #add custom safety buffer away from obs
        self.gap_min_width = self.car_width + self.safety_buffer
        
        self.obstacle_thresh = 1.5      #front detection for obstacles
        self.lookahead_angle = np.deg2rad(210)

        # --- Boost parameters ---
        self.boost_steer_threshold = 0.25      # Steering below this triggers boost
        self.max_boost_speed = 1.5            # Maximum extra speed
        self.boost_decay = 0.25               # How fast boost decays per cycle
        self.boost_increment = 0.25            # How fast boost increases when active
        self.current_boost = 0.0              # Internal boost state

        # --- Keyboard state ---
        self.stop_car = False
        self.shutdown_flag = False

        # Start keyboard thread
        self.kb_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.kb_thread.start()

    #detects when user presses key. X to stop, Q to quit
    def keyboard_listener(self):          
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while not self.shutdown_flag:
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    key = sys.stdin.read(1)
                    if key.lower() == 'x':
                        self.stop_car = not self.stop_car
                        state = "stopped" if self.stop_car else "resumed"
                        self.get_logger().info(f"Toggle command (x): car {state}.")
                    elif key.lower() == 'q':
                        self.get_logger().info("Shutdown command (q) received.")
                        self.shutdown_flag = True
                        rclpy.shutdown()
                        break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    #filters and scores gaps around car
    def scan_callback(self, scan: LaserScan):
        if self.shutdown_flag:
            return

        drive_msg = AckermannDriveStamped()

        if self.stop_car:
            drive_msg.drive.speed = 0.0
            drive_msg.drive.steering_angle = 0.0
            self.drive_pub.publish(drive_msg)
            return

        ranges = np.array(scan.ranges)
        valid = np.logical_and(ranges > scan.range_min, ranges < scan.range_max)
        ranges[~valid] = np.nan

        num_beams = len(ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, num_beams)
        forward_mask = np.abs(angles) <= self.lookahead_angle
        ranges = ranges[forward_mask]
        angles = angles[forward_mask]

        is_obstacle = ranges < self.obstacle_thresh
        is_free = np.logical_or(ranges > self.obstacle_thresh, np.isnan(ranges))

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

        best_segment = None
        best_score = -1
        for (s, e) in free_segments:
            width_angle = (e - s) * scan.angle_increment
            seg_ranges = ranges[s:e + 1]
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
            drive_msg.drive.speed = self.min_speed
            drive_msg.drive.steering_angle = 0.0
            self.drive_pub.publish(drive_msg)
            return

        s, e, bottleneck, width_angle = best_segment
        target_idx = (s + e) // 2
        target_angle = angles[target_idx]

        steering = np.clip(target_angle, -self.max_steering, self.max_steering)
        base_speed = self.max_speed - (abs(steering) / self.max_steering) * (self.max_speed - self.min_speed)

        # --- Boost mechanic ---
        if abs(steering) < self.boost_steer_threshold:
            self.current_boost = min(self.current_boost + self.boost_increment, self.max_boost_speed)
        else:
            self.current_boost = max(self.current_boost - self.boost_decay, 0.0)

        final_speed = base_speed + self.current_boost

        drive_msg.drive.speed = float(final_speed)
        drive_msg.drive.steering_angle = float(steering)
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DisparityBiggestGapFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
