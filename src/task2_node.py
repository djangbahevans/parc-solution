#! /usr/bin/env python
import sys

import cv2
import numpy as np
import rospy
from cv_bridge.core import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion

try:
    goal_x = float(sys.argv[1])
    goal_y = float(sys.argv[2])
except IndexError:
    rospy.logerr("usage: rosrun task2_node.py <goal_x> <goal_y>")
except ValueError as e:
    rospy.logfatal(f"{str(e)}")
    rospy.signal_shutdown("Fatal error")

cmd_msg = Twist()
cmd_msg.linear.x = 0.5
start = 0
end = 0


class Traffic:
    def __init__(self) -> None:
        rospy.init_node("task2_solution")

        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.image_cb)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.odom = (0, 0, 0)

    def image_cb(self, msg: Image):
        # Capture frame-by-frame
        captured_frame = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="bgr8")

        # First blur to reduce noise prior to color space conversion
        captured_frame_bgr = cv2.medianBlur(captured_frame, 3)
        # Convert to Lab color space, we only need to check one channel (a-channel) for red here
        captured_frame_hsv = cv2.cvtColor(
            captured_frame_bgr, cv2.COLOR_BGR2HSV)

        # Threshold the Lab image, keep only the green pixels
        captured_frame_green = cv2.inRange(
            captured_frame_hsv, np.array([40, 200, 200]), np.array([70, 255, 255]))

        # Second blur to reduce more noise, easier circle detection
        captured_frame_green = cv2.GaussianBlur(
            captured_frame_green, (5, 5), 2, 2)

        # Use the Hough transform to detect circles in the image
        circles = cv2.HoughCircles(captured_frame_green, cv2.HOUGH_GRADIENT, 1,
                                   captured_frame_green.shape[0] / 8, param1=20, param2=18, minRadius=5, maxRadius=25)

        # If we have extracted a circle, draw an outline
        # We only need to detect one circle here, since there will only be one reference object
        if circles is not None:
            self.image_sub.unregister()
            self.go_to_point((goal_x, goal_y), max_speed=1)

    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        rotation = msg.pose.pose.orientation
        (_, _, theta) = euler_from_quaternion(
            [rotation.x, rotation.y, rotation.z, rotation.w])

        self.odom = (x, y, theta)

    def go_to_point(self, point: "tuple[float, float]", max_speed: float = 1, distance: float = 0.5):
        """Navigates the robot straight to a point. Should only be used for short distances where there are no obstacles.

        Args:
            point (tuple[float, float]): The point to navigate to.
            distance (float, optional): The distance to assume destination reached. Defaults to 0.5.
        """
        cmd_vel = Twist()
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            del_x = point[0] - self.odom[0]
            del_y = point[1] - self.odom[1]
            norm = np.sqrt(del_x**2 + del_y**2)
            d_theta = np.arctan2(del_y, del_x)

            if norm < distance:
                self.stop()
                break
            cmd_vel.linear.x = (.5 * norm) if (.5 *
                                               norm) < max_speed else max_speed
            cmd_vel.angular.z = .5 * (d_theta - self.odom[2])
            self.vel_pub.publish(cmd_vel)
            rate.sleep()

    def stop(self):
        """Stops the robot
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = 0
        cmd_vel.angular.z = 0
        self.vel_pub.publish(cmd_vel)


if __name__ == "__main__":
    try:
        Traffic()

        rospy.spin()
    except rospy.ROSInterruptException:
        sys.exit(0)
