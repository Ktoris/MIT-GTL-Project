    #!/usr/bin/python3
    import os
    import rclpy
    from rclpy.node import Node

    from std_msgs.msg import Header
    from sensor_msgs.msg import Range
    from duckietown_msgs.msg import WheelsCmdStamped


    class scanNode(Node):
        def __init__(self):
            super().__init__('scan')
            self.vehicle_name = os.getenv('VEHICLE_NAME')
            self.wheels_pub = self.create_publisher(
                WheelsCmdStamped,
                f'/{self.vehicle_name}/wheels_cmd',
                10
            )

            # ===== SCAN PARAMETERS =====
            self.turn_speed = 0.3
            self.phase_time = 0.6  # seconds per direction

            # state
            self.phase = 0
            self.last_switch = self.get_clock().now()

            # timer at 20 Hz
            self.timer = self.create_timer(0.05, self.scan_step)

            self.get_logger().info("Scan node started")

        def run_wheels(self, frame_id, vel_left, vel_right):
            wheel_msg = WheelsCmdStamped()
            wheel_msg.header.stamp = self.get_clock().now().to_msg()
            wheel_msg.header.frame_id = frame_id
            wheel_msg.vel_left = vel_left
            wheel_msg.vel_right = vel_right
            self.wheels_pub.publish(wheel_msg)

        def scan_step(self):
            now = self.get_clock().now()
            elapsed = (now - self.last_switch).nanoseconds * 1e-9

            if elapsed > self.phase_time:
                self.phase = (self.phase + 1) % 3
                self.last_switch = now

            # 0 = left, 1 = right, 2 = stop (center)
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
        node = scanNode()
        rclpy.spin(node)
        rclpy.shutdown()

    if __name__ == "__main__":
        main()
