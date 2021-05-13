#!/usr/bin/env python
import sys

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from numpy.lib.function_base import average
from sensor_msgs.msg import Image


class LaneDetector(object):
    def __init__(self) -> None:
        rospy.init_node("lane_detector")
        self.bridge = CvBridge()
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb)
        self.pub = rospy.Publisher("/lanes", Image, queue_size=10)
        rospy.spin()

    def image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # First blur to reduce noise prior to color space conversion
        captured_frame_bgr = cv2.medianBlur(frame, 3)

        # Convert to HSV color space, we only need to check one channel (a-channel) for red here
        captured_frame_hsv = cv2.cvtColor(
            captured_frame_bgr, cv2.COLOR_BGR2HSV)

        # Threshold the Lab image, keep only the grey pixels
        captured_frame_lab_g = cv2.inRange(
            captured_frame_hsv, np.array([0, 0, 120]), np.array([40, 10, 128]))
        captured_frame_lab_w = cv2.inRange(
            captured_frame_hsv, np.array([0, 0, 245]), np.array([179, 255, 255]))

        captured_frame_lab_grey = cv2.bitwise_or(
            captured_frame_lab_g, captured_frame_lab_w)

        # Second blur to reduce more noise, easier circle detection
        captured_frame_lab_grey = cv2.GaussianBlur(
            captured_frame_lab_grey, (5, 5), 2, 2)
        retval, lanes = cv2.threshold(
            captured_frame_lab_grey, 128, 255, cv2.THRESH_BINARY)

        img = self.bridge.cv2_to_imgmsg(lanes)
        self.pub.publish(img)
        # cv2.imshow("lanes", lanes)
        # cv2.imshow("original", frame)
        # cv2.waitKey(1)


if __name__ == "__main__":
    try:
        LaneDetector()
    except rospy.ROSInterruptException:
        sys.exit()
