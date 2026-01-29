import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
from .img_rec import detect_color

class ColorRecognitionNode(Node):
    def __init__(self):
        super().__init__("color_recognition")
        self.publisher = self.create_publisher(String, "/detected_color", 10)
        self.timer = self.create_timer(0.5, self.process_image) # Scan 2 times per second
        self.get_logger().info("Vision Node Scanning...")

    def process_image(self):
        # In a real robot, use cv2.VideoCapture(0)
        frame = cv2.imread("test.jpg") 
        if frame is None:
            return

        result = detect_color(frame)
        msg = String()
        msg.data = result
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ColorRecognitionNode()
    rclpy.spin(node)
    rclpy.shutdown()