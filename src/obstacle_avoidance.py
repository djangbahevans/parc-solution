#!/usr/bin/env python
import sys
from functools import reduce
from math import inf

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from tf.listener import TransformListener
from tf.transformations import euler_from_quaternion

from Graph import Graph, Vertex


class ObstacleAvoidance(object):
    def __init__(self) -> None:
        super().__init__()

        rospy.init_node("obstacle_avoidance")
        self.scan_sub = rospy.Subscriber(
            "/scan", LaserScan, callback=self.scan_cb)
        self.odom_sub = rospy.Subscriber(
            "/odom", Odometry, callback=self.odom_cb)
        self.image_sub = rospy.Subscriber(
            "/lanes", Image, self.image_cb)
        self.bridge = CvBridge()

        self.vel_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.obstacle = False
        self.obstacles = []
        self.odom = {}
        self.stored_odom = {}
        self.lanes = np.zeros((480, 640))
        self.odom_start = False
        self.scan_start = False
        self.img_start = False

        self.graph = self.create_map()

        while (not (self.odom_start and self.scan_start and self.img_start)) and (not rospy.is_shutdown()):
            continue

        self.listener = TransformListener()
        self.listener.waitForTransform(
            "odom", "base_link", rospy.Time(), rospy.Duration(20))

        self.main()
        # rospy.spin()

    def main(self):
        """Main loop of robot
        """
        self.store_current_heading()
        while not rospy.is_shutdown():
            if not self.obstacle:
                self.keep_going(kind="lane")
                # self.return_to_former_heading(speed=0.5)
                self.check_for_obstacles()
            else:
                print("Obstacle!!!")
                (true_point_local, obs_offset_local) = self.determine_pos_of_obstacle()
                obs_offset_global = self.robot_to_global_frame(
                    obs_offset_local)
                true_point_global = self.robot_to_global_frame(
                    true_point_local)
                print(f"True point local: {true_point_local}")
                print(f"True point global: {true_point_global}")

                self.go_to_point(obs_offset_global)
                self.reset_obstacles()

                while not self.obstacle_behind(true_point_global, offset=0) and not rospy.is_shutdown():
                    self.return_to_former_heading(speed=0)
                    self.keep_going(kind="forward")
                else:
                    self.obstacle = False

        self.stop()

    def reset_obstacles(self):
        self.obstacles = []

    def global_to_robot_frame(self, p):
        """Converts a point in global frame (odom) to robot frame (base_link)

        Args:
            p (tuple[float, float]): Point in global frame to convert

        Returns:
            list[float, float]: Converted point in robot frame
        """
        point = PointStamped()
        point.header.frame_id = "odom"
        point.header.stamp = rospy.Time(0)
        point.point.x = p[0]
        point.point.y = p[1]
        point.point.z = 0.0
        transformed = self.listener.transformPoint("base_link", point)
        return (transformed.point.x, transformed.point.y)

    def robot_to_global_frame(self, p):
        """Converts a point from robot frame (base_link) to the global frame (odom)

        Args:
            p (tuple[float, float]): Point in robot frame to convert

        Returns:
            list[float, float]: Converted point in global frame
        """
        point = PointStamped()
        point.header.frame_id = "base_link"
        point.header.stamp = rospy.Time(0)
        point.point.x = p[0]
        point.point.y = p[1]
        point.point.z = 0.0
        transformed = self.listener.transformPoint("odom", point)

        return (transformed.point.x, transformed.point.y)

    def stop(self):
        """Stops the robot
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = 0
        cmd_vel.angular.z = 0
        self.vel_cmd.publish(cmd_vel)

    def obstacle_behind(self, point, offset=0):
        """Checks if the robot is in front of an obstacle point

        Args:
            point (tuple[float, float]): The point to check for
            offset (int, optional): How far behind the point should be for it to be detected. Defaults to 0.

        Returns:
            boolean: True if point is behind the robot
        """
        p = self.global_to_robot_frame(point)
        if p[0] >= (0 - offset):
            return False
        else:
            print("Beyond obstacle!")
            return True

    def return_to_former_heading(self, speed=0):
        """Returns the robot to the set heading
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = speed
        while abs(self.odom["theta"] - self.current_heading) > 0.01:
            err = (self.odom["theta"] - self.current_heading)
            cmd_vel.angular.z = -0.5 * err
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

    def at_coordinates(self, coordinate: "tuple[float, float]", scheme="y", radius=0.01) -> bool:
        """Checks if robot is at or near a particular coordinate.

        Args:
            coordinate (tuple[float, float]): The coordinate to check for.
            scheme (str, optional): Options are "x", "y" and "both". Used to determine how to calculate distance. "x" and "y" takes distance from global x and y axis respectively only into account, "both" calculates the norm from both axis. Defaults to "y".
            radius (float, optional): The radius distance from the to assume convergence. Defaults to 0.01.

        Returns:
            bool: Returns True if robot is within a specified distance from the point.
        """
        # Check if we have desired y
        if scheme == "y":
            return True if abs(self.odom["y"] - coordinate[1]) <= radius else False
        elif scheme == "x":
            return True if abs(self.odom["x"] - coordinate[0]) <= radius else False
        elif scheme == "both":
            return True if np.sqrt((self.odom["x"] - coordinate[0])**2 + (self.odom["y"] - coordinate[0])**2) <= radius else False

    def determine_pos_of_obstacle(self):
        """Determines the position of the obstacles and returns the nearest point

        Returns:
            tuple[list[float, float], list[float, float]]: return[0] represents the nearest obstacle postion. return[1] is ideal offset point for robot to go to
        """
        obs_points = list(
            filter(lambda x: x[1] >= -0.35 and x[1] <= 0.35, self.obstacles))  # Get points that collides with vehicle
        if self.average_point(*obs_points)[1] > 0:  # Obstacles on the left
            # return right most point
            true_obs_point: list[float] = np.array(
                min(self.obstacles, key=lambda x: x[1]))
            p = np.array(
                min(self.obstacles, key=lambda x: x[1])) + np.array([-1, -1])
            p = [p[0] - 2, -.4]
        else:  # Obstacles on the right
            # return left most point
            true_obs_point = np.array(max(self.obstacles, key=lambda x: x[1]))
            p = np.array(
                max(self.obstacles, key=lambda x: x[1])) + np.array([-1, 1])
            p: list[float] = [p[0] - 2, .4]

        return ((true_obs_point[0], true_obs_point[1]), (p[0], p[1]))

    def check_for_obstacles(self):
        """Checks for obstacles and sets self.obstacle to true if obstacle is found in the path of the robot.
        """
        for point in self.obstacles:
            if point[0] > 0 and point[0] < 1.5:
                if point[1] >= -0.35 and point[1] <= 0.35:
                    self.obstacle = True

    def store_current_heading(self):
        """Saves current heading for later retrieval
        """
        self.current_heading = self.odom["theta"]

    def keep_going(self, kind="lane"):
        """Follows the lane based on camera data

        Args:
            type (str, optional): The type of motion supported. Options are "lane" and "forward". Defaults to "lane".
        """
        cmd_vel = Twist()
        if kind == "forward":
            cmd_vel.linear.x = 0.5
            cmd_vel.angular.z = 0
            self.vel_cmd.publish(cmd_vel)
        elif kind == "lane":
            left = self.sliding_window(self.lanes, side="left")
            right = self.sliding_window(self.lanes, side="right")
            cmd_vel.linear.x = 0.5
            err = right - left
            cmd_vel.angular.z = np.clip(0.001 * err, -.05, .05)
            self.vel_cmd.publish(cmd_vel)
            if abs(err) < 1:
                self.store_current_heading()

    def odom_cb(self, msg: Odometry):
        """Handles odometry messages

        Args:
            msg (Odometry): Odometry data
        """
        position = msg.pose.pose.position
        rotation = msg.pose.pose.orientation
        (_, _, theta) = euler_from_quaternion(
            [rotation.x, rotation.y, rotation.z, rotation.w])
        self.odom["x"] = position.x
        self.odom["y"] = position.y
        self.odom["theta"] = theta
        self.odom_start = True

    def scan_cb(self, msg: LaserScan):
        """Handles LaserScan messages

        Args:
            msg (LaserScan): LaserScan data
        """
        angle_range = msg.angle_max - msg.angle_min
        msg_len = len(msg.ranges)

        for i in range(len(msg.ranges)):
            angle = i * angle_range/msg_len - msg.angle_max
            dist = msg.ranges[i]
            if dist != inf:
                y_dist = dist * np.sin(angle)
                x_dist = dist * np.cos(angle)
                self.obstacles.append((x_dist, y_dist))
        self.scan_start = True

    def image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        _, lanes = cv2.threshold(
            frame, 128, 255, cv2.THRESH_BINARY)
        self.lanes = lanes
        self.img_start = True

    def sliding_window(self, img, side: str = "left") -> float:
        """Tries to find the average position of a lane line, if none present, returns 479

        Args:
            img (np.ndarray): A binary image of the lane markings
            side (str, optional): The side of the image to check for. Defaults to "left".

        Returns:
            float: The average position of the lane marking on the side of the image.
        """
        if side == "left":
            section = img[:, 0]
        else:
            section = img[:, 639]

        done = False
        pixels = []
        for i in reversed(range(480)):
            if section[i] == 255:
                pixels.append(i)
                done = True
            elif done and section[i] == 0:
                break
        average = 479
        if len(pixels) != 0:
            average = sum(pixels) / len(pixels)
        return average

    def average_point(self, *points) -> "tuple[float, float]":
        """Calculates the average of points

        Returns:
            tuple[float, float]: (x, y) representing the average of all the points
        """
        length = len(points)
        sum_x = reduce(lambda total, point: total + point[0], points, 0)
        sum_y = reduce(lambda total, point: total + point[1], points, 0)
        return (sum_x/length, sum_y/length)

    def find_shortest_path(self, start: str, end: str, path=[]) -> "list[str]":
        """Finds the shortest path between two nodes on the graph

        Args:
            start (str): The starting position or node
            end (str): The destination position or node

        Returns:
            list[str]: A list of nodes to move through to get to the desired node
        """
        path = path + [start]
        if start == end:
            return path
        if not start in self.graph.vert_dict:
            return None
        shortest = None
        for node in self.graph.vert_dict[start].adjacent:
            if node.id not in path:
                newpath = self.find_shortest_path(node.id, end, path)
                if newpath:
                    if not shortest or len(newpath) < len(shortest):
                        shortest = newpath
        return shortest

    def nearest_node(self, p: "tuple[float, float]") -> Vertex:
        """Takes a point and returns the node closest to that point

        Args:
            p (tuple[float, float]): The point

        Returns:
            str: The node as a string
        """
        prev_dist = inf
        for node in self.graph:
            dist = self.distance(p, node.coordinates)
            if dist < prev_dist:
                nearest_node = node
                prev_dist = dist

        return nearest_node

    def distance(self, p1: "tuple[float, float]", p2: "tuple[float, float]") -> float:
        """Calculates the distance between two points

        Args:
            p1 (tuple[float, float]): The first point
            p2 (tuple[float, float]): The second point

        Returns:
            float: The scalar distance between the two points
        """
        return np.sqrt((p1[0] - p2[0])**2, (p1[1] - p2[1])**2)

    def create_map(self) -> Graph:
        g = Graph()
        g.add_vertex("A", (-30.5, 10.9))
        g.add_vertex("B", (-30.5, -10.9))
        g.add_vertex("C", (-12.2, 10.9))
        g.add_vertex("D", (-12.2, -10.9))
        g.add_vertex("E", (-2.0, 10.9))
        g.add_vertex("F", (-2.0, -10.9))
        g.add_vertex("G", (7.65, 10.9))

        g.add_edge("A", "B", "lane")
        g.add_edge("A", "C", "lane")

        g.add_edge("B", "D", "lane")

        g.add_edge("C", "D", "lane")
        g.add_edge("C", "E", "crossing")

        g.add_edge("D", "F", "crossing")

        g.add_edge("E", "F", "lane")
        g.add_edge("E", "G", "lane")

        return g


if __name__ == "__main__":
    try:
        ObstacleAvoidance()
    except rospy.ROSInterruptException:
        sys.exit()
