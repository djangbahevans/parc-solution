#!/usr/bin/env python
from math import inf, nan

import cv2
import numpy as np
import rospy
from cv_bridge.core import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan

from libraries.PID import PID

x_history = []
y_history = []


def get_slope(line):
    for x1, y1, x2, y2 in line:
        if x1 == x2:
            return inf
        return (y1-y2) / (x1-x2)


def get_y_intercept(line):
    # b = y - m*x
    for x1, y1, _, _ in line:
        return y1 - get_slope(line) * x1


def get_intercept(lines):
    intercepts_x = []
    intercepts_y = []
    for i in range(len(lines)-1):
        l1 = lines[i]
        m1 = get_slope(l1)
        b1 = get_y_intercept(l1)
        l2 = lines[i+1]
        m2 = get_slope(l2)
        if abs(m1 - m2) < 0.001 or m1 == None or m2 == None:
            continue
        b2 = get_y_intercept(l2)
        if b2 == inf or b2 == -inf or b2 == nan:
            continue
        x = (b2-b1)/(m1-m2)
        y = (b2*m1-b1*m2)/(m1-m2)
        # if x == inf or y == inf or x == -inf or y == -inf:
        #     continue
        intercepts_x.append(x)
        intercepts_y.append(y)

    if len(intercepts_x) == 0 or len(intercepts_y) == 0:
        return None

    return (int(np.average(intercepts_x)), int(np.average(intercepts_y)))


def merge_lines(lines):
    right_lines = []
    left_lines = []
    others_lines = []

    # 1. Filter lines into left, right and others
    for line in lines:
        slope = get_slope(line)
        if slope == inf:
            continue
        if abs(slope) < 0.05:
            others_lines.append(line)
        elif slope > 0:
            right_lines.append(line)
        elif slope < 0:
            left_lines.append(line)

    chosen_lines = {}
    left_avg = []
    right_avg = []
    others_avg = []
    for line in left_lines:
        for x1, _, x2, _ in line:
            left_avg.append((x1 + x2) / 2)
    for line in right_lines:
        for x1, _, x2, _ in line:
            right_avg.append((x1 + x2) / 2)
    for line in others_lines:
        for _, y1, _, y2 in line:
            others_avg.append((y1 + y2) / 2)

    # 2. For left lines, pick lines with the greatest x mean
    # 3. For right lines, pick lines with the smallest x mean
    # 4. For others, pick lines with the smallest y mean
    if len(left_avg) > 0:
        chosen_lines["l"] = left_lines[left_avg.index(max(left_avg))]
    if len(right_avg) > 0:
        chosen_lines["r"] = (right_lines[right_avg.index(min(right_avg))])
    # if len(others_avg) > 0:
    #     chosen_lines["o"] = (others_lines[others_avg.index(min(others_avg))])

    return chosen_lines


def image_cb(msg: Image):
    cmd_msg = Twist()
    cmd_msg.linear.x = 0.5

    turn_pid = PID(Kp=-.0005, Ki=0.001, Kd=0, setpoint=0)

    captured_frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    # captured_frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    output_frame = captured_frame.copy()

    # First blur to reduce noise prior to color space conversion
    captured_frame_bgr = cv2.medianBlur(captured_frame, 3)

    # Convert to HSV color space, we only need to check one channel (a-channel) for red here
    captured_frame_hsv = cv2.cvtColor(
        captured_frame_bgr, cv2.COLOR_BGR2HSV)

    # Threshold the Lab image, keep only the grey pixels
    captured_frame_lab_g = cv2.inRange(
        captured_frame_hsv, np.array([0, 0, 120]), np.array([40, 10, 128]))
    captured_frame_lab_w = cv2.inRange(
        captured_frame_hsv, np.array([0, 0, 245]), np.array([0, 0, 255]))
    
    captured_frame_lab_grey = cv2.bitwise_or(captured_frame_lab_g, captured_frame_lab_w)

    # Second blur to reduce more noise, easier circle detection
    captured_frame_lab_grey = cv2.GaussianBlur(
        captured_frame_lab_grey, (5, 5), 2, 2)

    canny = cv2.Canny(captured_frame_lab_grey, 1, 1)

    # Use the Hough transform to detect LINES in the image
    # lines = cv2.HoughLines(captured_frame_lab_grey, 1, np.pi/180, 200)
    lines = cv2.HoughLinesP(canny, rho=1, theta=np.pi /
                            720, threshold=50, minLineLength=50, maxLineGap=50)

    if lines is not None:
        # global x_history, y_history
        lines_dict = merge_lines(lines)
        lines = list(lines_dict.values())
        intercept = get_intercept(lines)

        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(output_frame, (x1, y1), (x2, y2),
                         (0, 0, 255), 1, cv2.LINE_AA)
        if intercept is not None:
            x_history.append(intercept[0])
            y_history.append(intercept[1])

            if len(x_history) > 5:
                del x_history[0]
            if len(y_history) > 5:
                del y_history[0]

            x = int(np.average(x_history))
            y = int(np.average(y_history))

            dis = int(640/2) - x

            cmd_msg.angular.z = turn_pid(dis)

            cv2.line(output_frame, (x, y), (int(640/2), 480),
                     (0, 255, 0), 1, cv2.LINE_AA)
            if "l" in lines_dict:
                p = lines_dict["l"][0]
                cv2.circle(output_frame, (p[2], p[3]), 20, (0, 0, 255))
            if "r" in lines_dict:
                p = lines_dict["r"][0]
                cv2.circle(output_frame, (p[0], p[1]), 20, (0, 0, 255))
        else:
            if "l" in lines_dict:
                cmd_msg.angular.z = -0.3
            elif "r" in lines_dict:
                cmd_msg.angular.z = 0.3

    else:
        cmd_msg.angular.z = 0

    # vel_pub.publish(cmd_msg)
    # print(f"VEL: {cmd_msg}")
    cv2.imshow('Output', captured_frame_lab_grey)
    cv2.waitKey(1)


def lidar_cb(msg: LaserScan):
    # Detection angle range:
    # 0 to -30 degrees front right
    # 0 to 30 degrees front left
    front_right = msg.ranges[169:201]
    front_left = msg.ranges[200:232]

    idx = msg.ranges.index(min(msg.ranges))
    # Angle at x = -180 + (idx * (360/400))
    # print(f"Index: {idx}")


if __name__ == "__main__":
    bridge = CvBridge()
    rospy.init_node("line_detector")
    rospy.Subscriber("/camera/color/image_raw", Image, image_cb)
    rospy.Subscriber("/scan", LaserScan, lidar_cb)
    vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.spin()
