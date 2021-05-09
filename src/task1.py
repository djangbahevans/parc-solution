#! /usr/bin/env python
import sys

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan


class Main:
    def __init__(self) -> None:
        self.bridge = CvBridge()
        rospy.init_node("task1_solution")
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb)
        rospy.Subscriber("/scan", LaserScan, self.lidar_cb)
        vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.spin()

    def lidar_cb(self, msg):
        pass

    def image_cb(self, msg):
        captured_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
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

        canny = cv2.Canny(captured_frame_lab_grey, 1, 1)

        M = cv2.moments(captured_frame_lab_grey)

        # calculate x,y coordinate of center
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        cv2.circle(output_frame, (cX, cY), 5, (255, 255, 255), -1)

        cv2.imshow("Hello", output_frame)
        cv2.waitKey(1)


if __name__ == "__main__":
    try:
        Main()
    except rospy.ROSInterruptException:
        sys.exit(0)
