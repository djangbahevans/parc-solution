#!/usr/bin/env python
import sys
from functools import reduce
from math import inf

import numpy as np
import rospy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.listener import TransformListener
from tf.transformations import euler_from_quaternion


class ObstacleAvoidance(object):
    def __init__(self) -> None:
        super().__init__()

        rospy.init_node("obstacle_avoidance")
        self.listener = TransformListener()
        self.listener.waitForTransform(
            "odom", "base_link", rospy.Time(), rospy.Duration(20))
        self.scan_sub = rospy.Subscriber(
            "/scan", LaserScan, callback=self.scan_cb)
        self.odom_sub = rospy.Subscriber(
            "/odom", Odometry, callback=self.odom_cb)

        self.vel_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.obstacle = False
        self.obstacles = []
        self.odom = {}
        self.stored_odom = {}

        self.main()

        rospy.spin()

    def main(self):
        while not rospy.is_shutdown():
            if not self.obstacle:
                print("No obstacles")
                self.keep_going()
                self.check_for_obstacles()
            else:
                self.store_current_heading()
                (true_point, obs_pos_local) = self.determine_pos_of_obstacle()
                obs_pos_global = self.robot_to_global_frame(obs_pos_local)
                true_point_global = self.robot_to_global_frame(true_point)

                self.go_to_point(obs_pos_global)

                # while not self.at_coordinates(point_robot):
                #     self.go_to_point(clear_point)
                #     if rospy.is_shutdown():
                #         break

                # self.return_to_former_heading()

                while not self.obstacle_behind(true_point_global, offset=1) and not rospy.is_shutdown():
                    self.return_to_former_heading()
                    self.keep_going(type="forward")
                else:
                    self.obstacle = False

    def global_to_robot_frame(self, p):
        point = PointStamped()
        point.header.frame_id = "odom"
        point.header.stamp = rospy.Time(0)
        point.point.x = p[0]
        point.point.y = p[1]
        point.point.z = 0.0
        transformed = self.listener.transformPoint("base_link", point)
        return [transformed.point.x, transformed.point.y]

    def robot_to_global_frame(self, p):
        point = PointStamped()
        point.header.frame_id = "base_link"
        point.header.stamp = rospy.Time(0)
        point.point.x = p[0]
        point.point.y = p[1]
        point.point.z = 0.0
        transformed = self.listener.transformPoint("odom", point)

        return [transformed.point.x, transformed.point.y]

    def stop(self):
        cmd_vel = Twist()
        self.vel_cmd.publish(cmd_vel)

    def obstacle_behind(self, point, offset=0):
        p = self.global_to_robot_frame(point)
        if p[0] > (0 - offset):
            return False
        else:
            print("beyond obstacle")
            return True

    def return_to_former_heading(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.5
        while abs(self.odom["theta"] - self.current_heading) > 0.01:
            cmd_vel.angular.z = -0.5 * \
                (self.odom["theta"] - self.current_heading)
            err = (self.odom["theta"] - self.current_heading)
            if rospy.is_shutdown():
                break
            self.vel_cmd.publish(cmd_vel)

    def go_to_point(self, point):
        """Sends robot to point

        Args:
            point (float, float): A point in global space
        """
        cmd_vel = Twist()
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            del_x = point[0] - self.odom["x"]
            del_y = point[1] - self.odom["y"]
            distance = np.sqrt(del_x**2 + del_y**2)
            d_theta = np.arctan2(del_y, del_x)

            cmd_vel.linear.x = .5 * distance if .5 * distance < 0.6 else 0.5
            cmd_vel.angular.z = .5 * (d_theta - self.odom["theta"])
            if distance < 0.5:
                break
            self.vel_cmd.publish(cmd_vel)
            rate.sleep()

    def at_coordinates(self, coordinate):
        # Check if we have desired y
        return True if abs(self.odom["y"] - coordinate[1]) <= 0.01 else False

    def determine_pos_of_obstacle(self):
        obs_points = list(
            filter(lambda x: x[1] >= -0.35 and x[1] <= 0.35, self.obstacles))  # Get points that collides with vehicle
        if self.average_point(*obs_points)[1] > 0:  # Obstacles on the left
            # return right most point
            true_obs_point = np.array(min(self.obstacles, key=lambda x: x[1]))
            p = np.array(
                min(self.obstacles, key=lambda x: x[1])) + np.array([-1, -1])
            p = [p[0] - 2, -.4]
        else:  # Obstacles on the right
            # return left most point
            true_obs_point = np.array(max(self.obstacles, key=lambda x: x[1]))
            p = np.array(
                max(self.obstacles, key=lambda x: x[1])) + np.array([-1, 1])
            p = [p[0] - 2, .4]

        return ([true_obs_point[0], true_obs_point[1]], [p[0], p[1]])

    def check_for_obstacles(self):
        for point in self.obstacles:
            if point[0] > 0:
                if point[1] >= -0.35 and point[1] <= 0.35:
                    self.obstacle = True

    def store_current_heading(self):
        theta = self.odom["theta"]
        self.current_heading = self.odom["theta"]

    def keep_going(self, type="lane"):
        """Follows the lane based on camera data
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.5
        cmd_vel.angular.x = 0
        self.vel_cmd.publish(cmd_vel)

    def odom_cb(self, msg: Odometry):
        position = msg.pose.pose.position
        rotation = msg.pose.pose.orientation
        (_, _, theta) = euler_from_quaternion(
            [rotation.x, rotation.y, rotation.z, rotation.w])
        self.odom["x"] = position.x
        self.odom["y"] = position.y
        self.odom["theta"] = theta

    def scan_cb(self, msg: LaserScan):
        angle_range = msg.angle_max - msg.angle_min
        msg_len = len(msg.ranges)

        for i in range(len(msg.ranges)):
            angle = i * angle_range/msg_len - msg.angle_max
            dist = msg.ranges[i]
            if dist != inf:
                y_dist = dist * np.sin(angle)
                x_dist = dist * np.cos(angle)
                self.obstacles.append((x_dist, y_dist))

    def average_point(self, *points):
        length = len(points)
        sum_x = reduce(lambda total, point: total + point[0], points, 0)
        sum_y = reduce(lambda total, point: total + point[1], points, 0)
        return (sum_x/length, sum_y/length)


if __name__ == "__main__":
    try:
        ObstacleAvoidance()
    except rospy.ROSInterruptException:
        sys.exit()
