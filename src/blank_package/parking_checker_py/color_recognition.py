#!/usr/bin/python3

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

import cv2
import numpy as np


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')

        self.vehicle_name = os.getenv('VEHICLE_NAME', 'duckiebot')

        self.create_subscription(
            CompressedImage,
            f'/{self.vehicle_name}/image/compressed',
            self.image_callback,
            10
        )

        self.get_logger().info("Color detector started")

    def image_callback(self, msg):
        # Convert compressed image to OpenCV format
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            return

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # ===== GREEN MASK =====
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)

        # ===== RED MASK (two ranges!) =====
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])

        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = red_mask1 | red_mask2

        # Count pixels
        green_pixels = cv2.countNonZero(green_mask)
        red_pixels = cv2.countNonZero(red_mask)

        # Threshold to avoid noise
        PIXEL_THRESHOLD = 3000

        if green_pixels > PIXEL_THRESHOLD:
            self.get_logger().info("GREEN detected")

        elif red_pixels > PIXEL_THRESHOLD:
            self.get_logger().info("RED detected")

        else:
            self.get_logger().info("No color detected")


def main():
    rclpy.init()
    node = ColorDetector()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()