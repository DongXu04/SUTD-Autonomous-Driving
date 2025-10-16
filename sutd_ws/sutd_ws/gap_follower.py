#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class GapFollowerFront(Node):
    """
    Pure forward-gap follower:
    - Only inspects a forward cone (configurable)
    - Finds free segments inside that cone (based on obstacle_thresh)
    - Estimates gap physical width from geometry and LIDAR distances
    - Picks the widest viable gap and steers to its center
    - Simple steering smoothing and speed scaling by steering magnitude
    """

    def __init__(self):
        super().__init__('gap_follower_front')

        # Publishers / subscribers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # --- Perception / gap parameters ---
        self.forward_cone_deg = 60.0    # half-angle of forward cone in degrees (so cone is ±60°)
        self.obstacle_thresh = 1.4      # range (m) above which we consider a ray "free"
        self.car_width = 0.5            # meters
        self.safety_buffer = 0.15       # additional clearance required
        self.gap_min_width = self.car_width + self.safety_buffer

        # --- Drive limits ---
        self.max_speed = 2.0
        self.min_speed = 0.6
        self.max_steering = 0.55        # radians, physical steering limit
        # smoothing
        self.max_steering_delta = 0.6   # maximum per-update change in steering (radians)
        self.steering_ema_alpha = 0.22  # EMA alpha (0..1). Higher -> faster reaction

        # --- Safety / fallback ---
        self.forward_emergency_dist = 0.8  # if something directly ahead closer than this, do evasive
        self.stop_on_no_gap = False        # if True, stop when no gap; if False, drive min_speed straight

        # state
        self.prev_steering = 0.0

        self.get_logger().info("GapFollowerFront ready — forward cone: ±{:.1f}°".format(self.forward_cone_deg))

    def scan_callback(self, scan: LaserScan):
        ranges = np.array(scan.ranges, dtype=float)
        # sanitize ranges
        invalid = np.isinf(ranges) | np.isnan(ranges) | (ranges <= 0.0)
        ranges[invalid] = scan.range_max

        # confine to forward cone
        half_cone = np.deg2rad(self.forward_cone_deg)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        forward_mask = (angles >= -half_cone) & (angles <= half_cone)
        if not np.any(forward_mask):
            self.get_logger().warn("No points in forward cone — publishing straight slow")
            self.publish_drive(0.0, self.min_speed)
            return

        f_indices = np.where(forward_mask)[0]
        f_ranges = ranges[f_indices]
        f_angles = angles[f_indices]

        # compute "free" boolean for forward sector
        is_free = f_ranges > self.obstacle_thresh

        # find contiguous free segments (indices relative to f_indices)
        free_segments = []
        N = len(is_free)
        in_free = False
        start = 0
        for i in range(N):
            if is_free[i] and not in_free:
                in_free = True
                start = i
            elif (not is_free[i] or i == N - 1) and in_free:
                end = i if not is_free[i] else i
                free_segments.append((start, end))
                in_free = False

        best_segment = None
        best_width = -1.0

        # evaluate segments: estimate physical width using bottleneck distance and angular width
        for (s, e) in free_segments:
            seg_ranges = f_ranges[s:e+1]
            # bottleneck - use conservative min range in segment (but ignore infinite range_max as large)
            bottleneck = float(np.nanmin(seg_ranges))
            width_angle = (e - s + 1) * scan.angle_increment  # radians
            gap_width_est = 2.0 * bottleneck * np.sin(width_angle / 2.0)
            if gap_width_est >= self.gap_min_width and gap_width_est > best_width:
                best_width = gap_width_est
                # compute center index in global frame (index into f_indices)
                center_local_idx = (s + e) // 2
                best_segment = {
                    's': s, 'e': e,
                    'center_local_idx': center_local_idx,
                    'bottleneck': bottleneck,
                    'width': gap_width_est,
                    'angle': f_angles[center_local_idx]
                }

        # Emergency: something very close straight ahead
        forward_idx = np.argmin(np.abs(f_angles))  # index closest to zero angle in forward arrays
        forward_dist = float(f_ranges[forward_idx])
        if forward_dist < self.forward_emergency_dist:
            # quick reactive steer away from the closer side: compare left/right means
            left_mean = float(np.nanmean(f_ranges[forward_idx:]))  # ahead + left half
            right_mean = float(np.nanmean(f_ranges[:forward_idx+1])) # right half includes forward
            steer = -self.max_steering if left_mean > right_mean else self.max_steering
            speed = self.min_speed
            self.publish_drive(steer, speed)
            return

        if best_segment is None:
            # no valid gap found
            if self.stop_on_no_gap:
                self.publish_drive(0.0, 0.0)
                self.get_logger().warn("No gap found in forward cone — stopping.")
            else:
                # keep going slowly straight (fallback)
                self.publish_drive(0.0, self.min_speed)
                self.get_logger().debug("No gap found — moving forward slowly.")
            return

        # We have a best gap: steer toward its center angle
        target_angle = float(best_segment['angle'])
        # steering target is basically the target_angle clamped to steering limits
        steering_cmd = np.clip(target_angle, -self.max_steering, self.max_steering)

        # speed schedule: slower when steering sharp, faster when steering straight & gap is wide
        steering_ratio = abs(steering_cmd) / (self.max_steering + 1e-6)
        # base speed scales with gap width (bigger gap -> more confident)
        width_conf = np.tanh(best_segment['width'] / (self.gap_min_width * 2.0))  # 0..~1
        speed = self.min_speed + (self.max_speed - self.min_speed) * ( (1.0 - steering_ratio) * 0.6 + 0.4 * width_conf )

        self.publish_drive(steering_cmd, speed)
        # debug
        self.get_logger().debug(f"Chosen gap width={best_segment['width']:.2f} m, angle={best_segment['angle']:.3f} rad, speed={speed:.2f}")

    def publish_drive(self, steering, speed):
        # Limit abrupt steering changes
        delta = steering - self.prev_steering
        if abs(delta) > self.max_steering_delta:
            steering = self.prev_steering + np.sign(delta) * self.max_steering_delta

        # Exponential moving average smoothing
        steering = self.steering_ema_alpha * steering + (1.0 - self.steering_ema_alpha) * self.prev_steering
        steering = float(np.clip(steering, -self.max_steering, self.max_steering))

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering
        drive_msg.drive.speed = float(np.clip(speed, self.min_speed, self.max_speed))
        self.drive_pub.publish(drive_msg)
        self.prev_steering = steering

def main(args=None):
    rclpy.init(args=args)
    node = GapFollowerFront()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
