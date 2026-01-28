import rospy
from geometry_msgs.msg import Twist
import time


class ScannerController:
    def __init__(self):
        rospy.init_node("scanner_controller")
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # ===== MOTION PARAMETERS (TUNE THESE) =====
        self.angular_speed = 1.2     # rad/s
        self.linear_speed = -0.05    # backward
        self.rotate_time = 1.2       # seconds
        self.center_time = 1.2
        self.back_time = 0.6

        rospy.sleep(1.0)
        rospy.loginfo("Scanner Controller started")
        self.scan_loop()

    def send_cmd(self, linear=0.0, angular=0.0, duration=1.0):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular

        start = time.time()
        rate = rospy.Rate(20)

        while time.time() - start < duration and not rospy.is_shutdown():
            self.pub.publish(twist)
            rate.sleep()

        self.stop()

    def stop(self):
        self.pub.publish(Twist())
        rospy.sleep
