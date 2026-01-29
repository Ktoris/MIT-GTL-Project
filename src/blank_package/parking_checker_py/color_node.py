import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2

from .img_rec import detect_color


class ColorNode(Node):
    def __init__(self):
        super().__init__("color_node")

        self.publisher = self.create_publisher(
            String,
            "/detected_color",
            10
        )

        # For now without camera, just with test image
        self.image_path = "test.jpg"
        self.process_image()

    def process_image(self):
        frame = cv2.imread(self.image_path)

        if frame is None:
            self.get_logger().error("Image not found")
            return

        detected, _, _ = detect_color(frame, color="red")

        msg = String()
        msg.data = detected
        self.publisher.publish(msg)

        self.get_logger().info(f"Detected color: {detected}")


def main(args=None):
    rclpy.init(args=args)
    node = ColorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
