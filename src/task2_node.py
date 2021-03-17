#! /usr/bin/env python
import sys
from math import atan2, pi, sqrt

import cv2
import numpy as np
import rospy
from cv_bridge.core import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion
from libraries.PID import PID

move = False

goal_x = float(sys.argv[1])
goal_y = float(sys.argv[2])

cmd_msg = Twist()
cmd_msg.linear.x = 0.5


def image_cb(msg: Image):
    # Capture frame-by-frame
    captured_frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # First blur to reduce noise prior to color space conversion
    captured_frame_bgr = cv2.medianBlur(captured_frame, 3)
    # Convert to Lab color space, we only need to check one channel (a-channel) for red here
    captured_frame_hsv = cv2.cvtColor(
        captured_frame_bgr, cv2.COLOR_BGR2HSV)

    # Threshold the Lab image, keep only the green pixels
    captured_frame_lab_red = cv2.inRange(
        captured_frame_hsv, np.array([40, 200, 200]), np.array([70, 255, 255]))

    # Second blur to reduce more noise, easier circle detection
    captured_frame_lab_red = cv2.GaussianBlur(
        captured_frame_lab_red, (5, 5), 2, 2)

    # Use the Hough transform to detect circles in the image
    circles = cv2.HoughCircles(captured_frame_lab_red, cv2.HOUGH_GRADIENT, 1,
                               captured_frame_lab_red.shape[0] / 8, param1=20, param2=18, minRadius=5, maxRadius=25)

    # If we have extracted a circle, draw an outline
    # We only need to detect one circle here, since there will only be one reference object
    if circles is not None:
        global move
        move = True
        image_sub.unregister()


def odom_cb(msg: Odometry):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rotation = msg.pose.pose.orientation
    (_, _, theta) = euler_from_quaternion(
        [rotation.x, rotation.y, rotation.z, rotation.w])
    theta = 2*pi + theta if theta < 0 else theta
    
    if move:
        global cmd_msg
        delta_x = goal_x - x
        delta_y = goal_y - y
        distance = sqrt(delta_x**2 + delta_y**2)
        if distance > 0.9:
            x_vel = distance_pid(distance)
            cmd_msg.linear.x = 0 if x_vel < 0.09 else x_vel
        else:
            cmd_msg.linear.x = 0
            cmd_msg.angular.z = 0
            vel_pub.publish(cmd_msg)
            
            rospy.loginfo("Target met")
            rospy.logwarn("Exiting...")
            rospy.signal_shutdown("Finished executing")

        d_theta = atan2(delta_y, delta_x)
        d_theta = 2*pi + d_theta if d_theta < 0 else d_theta
        angle_pid.setpoint = d_theta
        ang_vel = angle_pid(theta)
        cmd_msg.angular.z = 0 if abs(ang_vel < 0.1) else ang_vel

        vel_pub.publish(cmd_msg)


if __name__ == "__main__":
    try:
        rospy.init_node("task2_solution")
        bridge = CvBridge()

        angle_pid = PID(Kp=-.05, Ki=0.001, Kd=0, setpoint=0)
        distance_pid = PID(Kp=-.15, Ki=.0, Kd=0, setpoint=0,
                           output_limits=(-1, 1))

        image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, image_cb)
        odom_sub = rospy.Subscriber("/odom", Odometry, odom_cb)
        vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.spin()
    except rospy.ROSInterruptException:
        sys.exit(0)
