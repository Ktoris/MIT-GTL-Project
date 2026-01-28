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
        self.wheels_pub = self.create_publisher(WheelsCmdStamped, f'/{self.vehicle_name}/wheels_cmd', 10)

    def run_wheels(self, frame_id, vel_left, vel_right):
        wheel_msg = WheelsCmdStamped()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        wheel_msg.header = header
        wheel_msg.vel_left = vel_left
        wheel_msg.vel_right = vel_right
        self.wheels_pub.publish(wheel_msg)

def main():
    rclpy.init()
    scan = scanNode()
    rclpy.spin(scan)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
