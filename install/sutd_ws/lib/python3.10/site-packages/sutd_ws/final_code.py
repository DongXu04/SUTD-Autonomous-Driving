#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan, Joy
from ackermann_msgs.msg import AckermannDriveStamped


class HybridCarController(Node):
    def __init__(self):
        super().__init__('hybrid_car_controller')

        # --- ROS2 Interfaces ---
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # --- Shared Parameters ---
        self.max_speed = 1.5
        self.min_speed = 0.5
        self.max_steering = 0.55

        # --- Manual Control Config ---
        self.axis_throttle = 1   # Left stick vertical
        self.axis_steering = 3   # Right stick horizontal
        self.lb_button_index = 4 # LB = manual mode
        self.rb_button_index = 5 # RB = autonomous mode
        self.manual_speed_scale = 1.5
        self.manual_steering_scale = 0.20

        # --- Gap Follower Config ---
        self.car_width = 0.5
        self.safety_buffer = 0.2
        self.gap_min_width = self.car_width + self.safety_buffer
        self.obstacle_thresh = 1.0
        self.lookahead_angle = np.deg2rad(210)

        self.boost_steer_threshold = 0.1
        self.max_boost_speed = 0
        self.boost_decay = 0.25
        self.boost_increment = 0.1
        self.current_boost = 0.0

        # --- States ---
        self.mode = "idle"  # "manual" or "auto"
        self.latest_joy = None
        self.latest_scan = None

        self.get_logger().info("HybridCarController ready. Hold LB for manual, RB for autonomous mode.")

    # =========================================================
    # JOYSTICK CALLBACK
    # =========================================================
    def joy_callback(self, msg: Joy):
        self.latest_joy = msg

        lb_pressed = msg.buttons[self.lb_button_index] if len(msg.buttons) > self.lb_button_index else 0
        rb_pressed = msg.buttons[self.rb_button_index] if len(msg.buttons) > self.rb_button_index else 0

        if lb_pressed:
            self.mode = "manual"
        elif rb_pressed:
            self.mode = "auto"
        else:
            self.mode = "idle"

        if self.mode == "manual":
            self.handle_manual_mode(msg)
        elif self.mode == "idle":
            self.stop_car()

    # =========================================================
    # MANUAL DRIVING
    # =========================================================
    def handle_manual_mode(self, msg: Joy):
        if len(msg.axes) <= max(self.axis_throttle, self.axis_steering):
            self.get_logger().warn("Joystick message missing expected axes.")
            return

        throttle = -msg.axes[self.axis_throttle]  # Invert axis: up = forward
        steering = msg.axes[self.axis_steering]

        speed = np.clip(-throttle * self.manual_speed_scale, -self.manual_speed_scale, self.manual_speed_scale)
        steering_angle = np.clip(steering * self.manual_steering_scale, -self.manual_steering_scale, self.manual_steering_scale)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = float(speed)
        drive_msg.drive.steering_angle = float(steering_angle)
        self.drive_pub.publish(drive_msg)

        self.get_logger().info(f"[MANUAL] Speed: {speed:.2f}, Steering: {steering_angle:.3f}")

    # =========================================================
    # AUTONOMOUS GAP FOLLOWER
    # =========================================================
    def scan_callback(self, scan: LaserScan):
        self.latest_scan = scan

        if self.mode != "auto":
            return  # Only act when in autonomous mode

        ranges = np.array(scan.ranges)
        valid = np.logical_and(ranges > scan.range_min, ranges < scan.range_max)
        ranges[~valid] = np.nan

        num_beams = len(ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, num_beams)
        forward_mask = np.abs(angles) <= self.lookahead_angle
        ranges = ranges[forward_mask]
        angles = angles[forward_mask]

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

        drive_msg = AckermannDriveStamped()

        if best_segment is None:
            self.get_logger().warn("[AUTO] No safe gap found. Slowing down.")
            drive_msg.drive.speed = self.min_speed
            drive_msg.drive.steering_angle = 0.0
        else:
            s, e, bottleneck, width_angle = best_segment
            target_idx = (s + e) // 2
            target_angle = angles[target_idx]

            steering = np.clip(target_angle, -self.max_steering, self.max_steering)
            base_speed = self.max_speed - (abs(steering) / self.max_steering) * (self.max_speed - self.min_speed)

            if abs(steering) < self.boost_steer_threshold:
                self.current_boost = min(self.current_boost + self.boost_increment, self.max_boost_speed)
            else:
                self.current_boost = max(self.current_boost - self.boost_decay, 0.0)

            final_speed = base_speed + self.current_boost

            drive_msg.drive.speed = float(final_speed)
            drive_msg.drive.steering_angle = float(steering)
            self.get_logger().info(f"[AUTO] Speed: {final_speed:.2f}, Steering: {steering:.2f}")

        self.drive_pub.publish(drive_msg)

    # =========================================================
    # STOP FUNCTION
    # =========================================================
    def stop_car(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0.0
        drive_msg.drive.steering_angle = 0.0
        self.drive_pub.publish(drive_msg)
        self.get_logger().info("[IDLE] Car stopped.")


def main(args=None):
    rclpy.init(args=args)
    node = HybridCarController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
