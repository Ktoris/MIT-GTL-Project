#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from duckietown_msgs.msg import WheelsCmdStamped
import cv2
import numpy as np

class ScanWithObjectDetection(Node):
    def __init__(self):
        super().__init__('scan_object_detection')

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

        # ===== COLOR & OBJECT DETECTION STATE =====
        self.color_detected = None      # 'red', 'green', or None
        self.object_detected = False

        # Timer for movement
        self.timer = self.create_timer(0.05, self.movement_step)

        self.get_logger().info("Scan + Object Detection node started")

    # ---------- WHEELS ----------
    def run_wheels(self, frame_id, vel_left, vel_right):
        msg = WheelsCmdStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.vel_left = vel_left
        msg.vel_right = vel_right
        self.wheels_pub.publish(msg)

    # ---------- COLOR & OBJECT DETECTION ----------
    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.color_detected = None
            self.object_detected = False
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        kernel = np.ones((5, 5), np.uint8)

        # ===== MASKS =====

        # ===== GREEN MASK =====
        green_mask = cv2.inRange(
            hsv,
            np.array([40, 50, 50]),
            np.array([80, 255, 255])
        )

        green_pixels = cv2.countNonZero(green_mask)
        GREEN_PIXEL_THRESHOLD = 4000

        # Red (wraparound)
        r_low1 = np.array([0, 120, 100])
        r_high1 = np.array([10, 255, 255])
        r_low2 = np.array([170, 120, 100])
        r_high2 = np.array([180, 255, 255])
        red_mask = (
                cv2.inRange(hsv, r_low1, r_high1) |
                cv2.inRange(hsv, r_low2, r_high2)
        )

        # Black
        black_mask = cv2.inRange(
            hsv, np.array([0, 0, 0]), np.array([180, 255, 75])
        )
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel)

        # ===== FIND OBJECTS =====
        red_objs = []
        black_objs = []

        contours_r, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours_r:
            if cv2.contourArea(c) > 1000:
                x, y, w, h = cv2.boundingRect(c)
                red_objs.append((x, y, w, h, x + w // 2, y + h // 2))

        contours_b, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours_b:
            if cv2.contourArea(c) > 1000:
                x, y, w, h = cv2.boundingRect(c)
                black_objs.append((x, y, w, h, x + w // 2, y + h // 2))

        # ===== UPDATE COLOR STATE =====
        if green_pixels > GREEN_PIXEL_THRESHOLD:
            self.color_detected = 'green'
        elif red_objs:
            self.color_detected = 'red'
        elif black_objs:
            self.color_detected = 'black'
        else:
            self.color_detected = None

        # ===== RED–BLACK–RED SANDWICH DETECTION =====
        self.object_detected = False

        for b in black_objs:
            bx, by = b[4], b[5]

            for i in range(len(red_objs)):
                for j in range(i + 1, len(red_objs)):
                    r1x, r1y = red_objs[i][4], red_objs[i][5]
                    r2x, r2y = red_objs[j][4], red_objs[j][5]

                    horiz = (
                            min(r1x, r2x) < bx < max(r1x, r2x)
                            and abs(r1y - r2y) < 100
                    )

                    vert = (
                            min(r1y, r2y) < by < max(r1y, r2y)
                            and abs(r1x - r2x) < 100
                    )

                    if horiz or vert:
                        self.object_detected = True
                        break
                if self.object_detected:
                    break
            if self.object_detected:
                break

        self.get_logger().info(
            f"Color: {self.color_detected}, Object: {self.object_detected}"
        )

    # ---------- MOVEMENT ----------
    def movement_step(self):
        # MOVE FORWARD ONLY IF RED + OBJECT
        if self.color_detected == 'red' and self.object_detected:
            self.run_wheels("forward", 0.4, 0.4)
            return

        # OTHERWISE: KEEP SCANNING
        now = self.get_clock().now()
        elapsed = (now - self.last_switch).nanoseconds * 1e-9
        if elapsed > self.phase_time:
            self.phase = (self.phase + 1) % 3
            self.last_switch = now

        if self.phase == 0:
            self.run_wheels("scan", -self.turn_speed, self.turn_speed)
        elif self.phase == 1:
            self.run_wheels("scan", self.turn_speed, -self.turn_speed)
        else:
            self.run_wheels("scan", 0.0, 0.0)


def main():
    rclpy.init()
    node = ScanWithObjectDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.run_wheels("shutdown", 0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
