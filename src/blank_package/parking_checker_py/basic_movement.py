import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import time


class ScannerController(Node):
    def __init__(self):
        super().__init__("scanner_controller")

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # ===== MOTION PARAMETERS (TUNE THESE) =====
        self.angular_speed = 1.2     # rad/s
        self.linear_speed = -0.05    # backward
        self.rotate_time = 1.2       # seconds
        self.center_time = 1.2
        self.back_time = 0.6

        time.sleep(1.0)
        self.get_logger().info("Scanner Controller started")

        self.scan_loop()

    def send_cmd(self, linear=0.0, angular=0.0, duration=1.0):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular

        start = time.time()
        rate_hz = 20
        period = 1.0 / rate_hz

        while time.time() - start < duration and rclpy.ok():
            self.pub.publish(twist)
            time.sleep(period)

        self.stop()

    def stop(self):
        self.pub.publish(Twist())
        time.sleep(0.1)

    def scan_loop(self):
        # Example motion sequence (adapt as needed)
        self.send_cmd(angular=self.angular_speed, duration=self.rotate_time)
        self.send_cmd(duration=self.center_time)
        self.send_cmd(linear=self.linear_speed, duration=self.back_time)


def main(args=None):
    rclpy.init(args=args)
    node = ScannerController()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
