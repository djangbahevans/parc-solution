#!/usr/bin/env python
import sys

import cv2
import numpy as np
import rospy
from cv_bridge.core import CvBridge
from sensor_msgs.msg import Image


def image_cb(msg):
    # Capture frame-by-frame
    captured_frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    output_frame = captured_frame.copy()

    # Convert original image to BGR, since Lab is only available from BGR
    # captured_frame_bgr = cv2.cvtColor(captured_frame, cv2.COLOR_BGRA2BGR)
    # First blur to reduce noise prior to color space conversion
    captured_frame_bgr = cv2.medianBlur(captured_frame, 3)
    # Convert to Lab color space, we only need to check one channel (a-channel) for red here
    captured_frame_lab = cv2.cvtColor(
        captured_frame_bgr, cv2.COLOR_BGR2HSV)
    # Threshold the Lab image, keep only the red pixels
    # captured_frame_lab_red = cv2.inRange(
    #     captured_frame_lab, np.array([0, 200, 200]), np.array([10, 255, 255]))
    # Threshold the Lab image, keep only the green pixels
    captured_frame_lab_red = cv2.inRange(
        captured_frame_lab, np.array([40, 200, 200]), np.array([70, 255, 255]))
    # Second blur to reduce more noise, easier circle detection
    captured_frame_lab_red = cv2.GaussianBlur(
        captured_frame_lab_red, (5, 5), 2, 2)
    # Use the Hough transform to detect circles in the image
    circles = cv2.HoughCircles(captured_frame_lab_red, cv2.HOUGH_GRADIENT, 1,
                               captured_frame_lab_red.shape[0] / 8, param1=20, param2=18, minRadius=5, maxRadius=25)

# If we have extracted a circle, draw an outline
# We only need to detect one circle here, since there will only be one reference object
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        cv2.circle(output_frame, center=(
            circles[0, 0], circles[0, 1]), radius=circles[0, 2], color=(0, 0, 255), thickness=2)

    else:
        cv2.putText(output_frame, "Hello World!!!", (200, 200),
                    cv2.FONT_HERSHEY_SIMPLEX, 2, 255)

    # Display the resulting frame, quit with q
    # cv2.imshow('frame', captured_frame_lab_red)
    cv2.imshow('frame', output_frame)
    cv2.waitKey(1)


# When everything done, release the capture
# cap.release()


if __name__ == "__main__":
    try:
        bridge = CvBridge()
        rospy.init_node("traffic_detector")
        rospy.Subscriber("/camera/color/image_raw", Image, image_cb)
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        sys.exit(0)
