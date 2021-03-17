#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from cv_bridge.core import CvBridge
from sensor_msgs.msg import Image


def image_cb(msg: Image):
    captured_frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    # captured_frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    output_frame = captured_frame.copy()

    # First blur to reduce noise prior to color space conversion
    captured_frame_bgr = cv2.medianBlur(captured_frame, 3)

    # Convert to HSV color space, we only need to check one channel (a-channel) for red here
    captured_frame_hsv = cv2.cvtColor(
        captured_frame_bgr, cv2.COLOR_BGR2HSV)

    # Threshold the Lab image, keep only the grey pixels
    captured_frame_lab_grey = cv2.inRange(
        captured_frame_hsv, np.array([0, 0, 120]), np.array([40, 10, 128]))

    # Second blur to reduce more noise, easier circle detection
    captured_frame_lab_grey = cv2.GaussianBlur(
        captured_frame_lab_grey, (5, 5), 2, 2)

    # Use the Hough transform to detect LINES in the image
    # lines = cv2.HoughLines(captured_frame_lab_grey, 1, np.pi/180, 200)
    lines = cv2.HoughLinesP(captured_frame_lab_grey, rho=1, theta=np.pi /
                            180, threshold=100, minLineLength=100, maxLineGap=50)

    if lines is not None:
        for i in range(0, len(lines)):
            l = lines[i][0]
            cv2.line(output_frame, (l[0], l[1]),
                     (l[2], l[3]), (0, 0, 255), 3, cv2.LINE_AA)

    cv2.imshow('Output', output_frame)
    # cv2.imshow('frame', captured_frame_bgr)
    cv2.imshow('Thresh', captured_frame_lab_grey)
    cv2.waitKey(1)


if __name__ == "__main__":
    bridge = CvBridge()
    rospy.init_node("line_detector")
    rospy.Subscriber("/camera/color/image_raw", Image, image_cb)
    rospy.spin()
