import cv2
import numpy as np

def detect_color(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Red has two ranges
    red_mask = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255])) + \
               cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))

    # Green range
    green_mask = cv2.inRange(hsv, np.array([35, 100, 100]), np.array([85, 255, 255]))

    red_area = mask_area(red_mask)
    green_area = mask_area(green_mask)

    if red_area > 1000:
        return "red"
    elif green_area > 1000:
        return "green"
    return "none"
def mask_area(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return sum(cv2.contourArea(cnt) for cnt in contours)