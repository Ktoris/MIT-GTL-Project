import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
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
