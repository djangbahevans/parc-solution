#!/usr/bin/env python
import logging
import math

import cv2
import cv_bridge
import matplotlib.pyplot as plt
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

logging.basicConfig(level=logging.DEBUG)


n = 0

def detect_edges(frame):
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(img, (3, 3), 0)
    ret, thresh = cv2.threshold(blur, 120, 255, cv2.THRESH_BINARY)
    edge = cv2.Canny(thresh, 100, 200)
    return edge


def region_of_interest(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)

    vertices = np.array([[0, height], [width, height], [
                        width, 276], [0, 255]], dtype=np.int32)
    cv2.fillConvexPoly(mask, vertices, 1)
    cropped = cv2.bitwise_and(edges, edges, mask=mask)
    rho = 1  # distance precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
    min_threshold = 10  # minimal of votes
    line_segments = cv2.HoughLinesP(
        cropped, rho, angle, min_threshold, np.array([]), minLineLength=8, maxLineGap=4)
    return line_segments


def average_slope_intercept(frame, line_segments):
    """
    This function combines line segments into one or two lane lines
    If all line slopes are < 0: then we only have detected left lane
    If all line slopes are > 0: then we only have detected right lane
    """
    lane_lines = []
    if line_segments is None:
        logging.info('No line_segment segments detected')
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1/3
    # left lane line segment should be on left 2/3 of the screen
    left_region_boundary = width * (1 - boundary)
    # right lane line segment should be on left 2/3 of the screen
    right_region_boundary = width * boundary

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                logging.info(
                    'skipping vertical line segment (slope=inf): %s' % line_segment)
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]
    logging.debug('lane lines: %s' % lane_lines)

    return lane_lines


def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 - 204)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]


def detect_lane(frame):
    edges = detect_edges(frame)
    line_segments = region_of_interest(edges)
    # line_segments = detect_line_segments(cropped_edges)
    lane_lines = average_slope_intercept(frame, line_segments)

    return lane_lines


def display_lines(frame, lines, line_color=(0, 255, 0), line_width=10):
    line_image = np.zeros_like(frame)
    height, width, _ = line_image.shape
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2),
                         line_color, line_width)
        steering_angle = 90  # can change depending on degree of turn
        steering_angle_radian = steering_angle / 180.0 * math.pi
        x3 = int(width / 2)
        y3 = height
        x4 = int(x3 - (height - 204) / math.tan(steering_angle_radian))
        y4 = int((height - 204))
        cv2.line(line_image, (x3, y3), (x4, y4), (255, 0, 0), line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image


def image_cb(imgmsg):
    frame = bridge.imgmsg_to_cv2(imgmsg, "bgr8")

    # frame = cv2.imread(
#     "/home/scripter/Documents/experimental/parc robotics/image.jpg")
    lane_lines = detect_lane(frame)
    # print(lane_lines)
    lane_lines_image = display_lines(frame, lane_lines)
    # plt.figure(figsize=(10, 10))
    # plt.imshow(lane_lines_image, cmap="gray")
    # plt.show()
    # print(lane_lines_image)
    global n
    cv2.imwrite(f"./traffic-{n}.jpg", frame)
    n += 1
    cv2.imshow("Image", lane_lines_image)
    cv2.waitKey(3)


if __name__ == "__main__":
    bridge = CvBridge()
    rospy.init_node("line_detector")
    rospy.Subscriber("/camera/color/image_raw", Image, image_cb)
    rospy.spin()
