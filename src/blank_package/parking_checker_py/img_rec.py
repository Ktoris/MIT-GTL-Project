import cv2
import numpy as np

def detect_color(frame, color='red'):
    """
    Returns:
        detected: 'red' or 'none'
        mask: binary mask of detected color
        frame_with_box: original frame with rectangles
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    if color == 'red':
        # Red has two HSV ranges
        lower1 = np.array([0, 100, 100])
        upper1 = np.array([10, 255, 255])
        lower2 = np.array([160, 100, 100])
        upper2 = np.array([179, 255, 255])
        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask = mask1 + mask2

    elif color == 'white':
        # White HSV range
        lower1 = np.array([0, 0, 200])
        upper1 = np.array([179, 50, 255])
        mask = cv2.inRange(hsv, lower1, upper1)

    else:
        raise ValueError("Unsupported color. Choose 'red' or 'white'.")

    mask = cv2.medianBlur(mask, 5)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detected = 'none'
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 500:  # filter small blobs
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, f"{color.upper()} DETECTED!", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            detected = color

    return detected, mask, frame

if __name__ == '__main__':
    image_path = 'test.jpg'  # Replace with your image path
    frame = cv2.imread(image_path)

    if frame is None:
        print(f"Failed to load image: {image_path}")
        exit()

    color_to_detect = 'red'  # Change to 'white' if you want
    detected, mask, frame_with_box = detect_color(frame, color=color_to_detect)

    print(f"Detected color: {detected}")

    cv2.imshow("Original with boxes", frame_with_box)
    cv2.imshow(f"{color_to_detect.capitalize()} Mask", mask)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
