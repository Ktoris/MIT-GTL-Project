#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from duckietown_msgs.msg import WheelsCmdStamped
import cv2
import numpy as np

class ScanAndFollow(Node):
    def __init__(self):
        super().__init__('scan_and_follow')

        self.vehicle_name = os.getenv('VEHICLE_NAME', 'duckiebot')

        # Publisher for wheels
        self.wheels_pub = self.create_publisher(
            WheelsCmdStamped,
            f'/{self.vehicle_name}/wheels_cmd',
            10
        )

        # Subscribe to camera
        self.create_subscription(
            CompressedImage,
            f'/{self.vehicle_name}/image/compressed',
            self.image_callback,
            10
        )

        # ===== SCAN PARAMETERS =====
        self.turn_speed = 0.3       # rad/s
        self.phase_time = 0.6       # seconds per direction
        self.phase = 0
        self.last_switch = self.get_clock().now()

        # ===== COLOR DETECTION STATE =====
        self.target_detected = None  # 'red', 'green', or None

        # Timer for movement
        self.timer = self.create_timer(0.05, self.movement_step)

        self.get_logger().info("Scan + Follow node started")

    # ---------- WHEELS ----------
    def run_wheels(self, frame_id, vel_left, vel_right):
        msg = WheelsCmdStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.vel_left = vel_left
        msg.vel_right = vel_right
        self.wheels_pub.publish(msg)

    # ---------- COLOR DETECTION ----------
    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.target_detected = None
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # GREEN mask
        green_mask = cv2.inRange(hsv, np.array([40,50,50]), np.array([80,255,255]))
        # RED mask (wrap around hue)
        red_mask1 = cv2.inRange(hsv, np.array([0,70,50]), np.array([10,255,255]))
        red_mask2 = cv2.inRange(hsv, np.array([170,70,50]), np.array([180,255,255]))
        red_mask = red_mask1 | red_mask2

        PIXEL_THRESHOLD = 3000
        green_pixels = cv2.countNonZero(green_mask)
        red_pixels = cv2.countNonZero(red_mask)

        if green_pixels > PIXEL_THRESHOLD:
            self.target_detected = 'green'
            self.get_logger().info("GREEN detected")
        elif red_pixels > PIXEL_THRESHOLD:
            self.target_detected = 'red'
            self.get_logger().info("RED detected")
        else:
            self.target_detected = None

    # ---------- MOVEMENT ----------
    def movement_step(self):
        # If color detected → move forward
        if self.target_detected is not None:
            # simple forward speed
            self.run_wheels("follow", 0.2, 0.2)
            return

        # Otherwise, do scanning pattern
        now = self.get_clock().now()
        elapsed = (now - self.last_switch).nanoseconds * 1e-9
        if elapsed > self.phase_time:
            self.phase = (self.phase + 1) % 3
            self.last_switch = now

        if self.phase == 0:
            # rotate left
            self.run_wheels("scan", -self.turn_speed, self.turn_speed)
        elif self.phase == 1:
            # rotate right
            self.run_wheels("scan", self.turn_speed, -self.turn_speed)
        else:
            # stop briefly
            self.run_wheels("scan", 0.0, 0.0)


def main():
    rclpy.init()
    node = ScanAndFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.run_wheels("shutdown", 0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
