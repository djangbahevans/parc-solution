#!/usr/bin/env python
import sys
from functools import reduce

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from tf.listener import TransformListener
from tf.transformations import euler_from_quaternion

from Graph import Graph

try:
    goal_x = float(sys.argv[1])
    goal_y = float(sys.argv[2])
except IndexError:
    rospy.logerr("usage: rosrun obstacle_avoidance.py <goal_x> <goal_y>")
except ValueError as e:
    rospy.logfatal(str(e))
    rospy.signal_shutdown("Fatal error")


class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node("task1_solution")
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

    def main(self):
        """Main loop of robot
        """
        first_point = self.find_first_point()
        (goal_node, _) = self.nearest_node((goal_x, goal_y))
        path = self.find_shortest_path(
            first_point.get_id(), goal_node.get_id())
        rospy.loginfo("Using path" + path)
        rospy.loginfo("Moving to " + first_point.get_id())
        self.go_to_intersection(first_point.coordinates)

        for i in range(1, len(path)):
            p = path[i]
            v = self.graph.get_vertex(p)
            h = np.arctan2(
                (v.coordinates[1] - self.odom["y"]), (v.coordinates[0] - self.odom["x"]))
            self.turn_to_heading(h)
            self.stop()
            rospy.loginfo("Moving from " + path[i-1] + " to p")
            self.go_to_intersection(v.coordinates)

        rospy.logwarn("At destination")

    def go_to_intersection(self, p):
        """Knows how to navigate to any intersection at point p

        Args:
            p (tuple[float, float]): Coordinates of the intersection
        """
        self.store_current_heading()
        while (not rospy.is_shutdown()) and (not self.at_coordinates(p, radius=2, scheme="both")):
            if not self.obstacle:
                self.keep_going(kind="lane", speed=1)
                self.check_for_obstacles()
            else:
                self.stop()
                (true_obs, obs_offset_local) = self.determine_pos_of_obstacle()
                obs_offset_global = self.robot_to_global_frame(
                    obs_offset_local)
                true_obs_global = self.robot_to_global_frame(
                    true_obs)
                rospy.logwarn("Obstacle at " + true_obs_global)

                self.go_to_point(obs_offset_global)
                self.reset_obstacles()

                while not self.obstacle_behind(obs_offset_global, offset=0.1) and not rospy.is_shutdown():
                    self.turn_to_heading(self.current_heading, speed=0)
                    self.keep_going(kind="forward")
                else:
                    self.obstacle = False

        self.go_to_point(p, distance=0.1)

    def find_first_point(self):
        """Find nodes in front of bot, then find the closest one.

        Returns:
            Vertex: The closest node to the robot
        """
        nodes = []
        for node in self.graph:
            p = self.global_to_robot_frame(node.coordinates)
            if p[0] >= 2:  # Ignore current node if there
                nodes.append(node)

        (node, _) = self.nearest_node((self.odom["x"], self.odom["y"]), nodes)
        return node

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
            return True

    def turn_to_heading(self, heading, speed = 0):
        """Returns the robot to the set heading

        Args:
            heading (float): The heading to turn to
            speed (float, optional): The forward speed of the robot. Defaults to 0.
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = speed
        while abs(self.odom["theta"] - heading) > 0.01:
            err = (self.odom["theta"] - heading)
            cmd_vel.angular.z = -0.5 * err
            if rospy.is_shutdown():
                break
            self.vel_cmd.publish(cmd_vel)

    def go_to_point(self, point, distance = 0.5):
        """Navigates the robot straight to a point. Should only be used for short distances where there are no obstacles.

        Args:
            point (tuple[float, float]): The point to navigate to.
            distance (float, optional): The distance to assume destination reached. Defaults to 0.5.
        """
        cmd_vel = Twist()
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            del_x = point[0] - self.odom["x"]
            del_y = point[1] - self.odom["y"]
            norm = np.sqrt(del_x**2 + del_y**2)
            d_theta = np.arctan2(del_y, del_x)

            cmd_vel.linear.x = .5 * norm if .5 * norm < 0.6 else 0.5
            cmd_vel.angular.z = .5 * (d_theta - self.odom["theta"])
            if norm < distance:
                self.stop()
                break
            self.vel_cmd.publish(cmd_vel)
            rate.sleep()

    def at_coordinates(self, coordinate, scheme="y", radius=0.01):
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
            return True if np.sqrt((self.odom["x"] - coordinate[0])**2 + (self.odom["y"] - coordinate[1])**2) <= radius else False

    def determine_pos_of_obstacle(self):
        """Determines the position of the obstacles and returns the nearest point

        Returns:
            tuple[list[float, float], list[float, float]]: return[0] represents the nearest obstacle postion. return[1] is ideal offset point for robot to go to
        """
        obs_points = list(
            filter(lambda p: p[1] >= -0.35 and p[1] <= 0.35, self.obstacles))  # Get points that collides with vehicle

        if self.average_point(*obs_points)[1] > 0:  # Obstacles on the left
            # return right most point
            true_obs_point = min(obs_points, key=lambda x: x[1])
            obs_offset = [true_obs_point[0], true_obs_point[1] - .75]
        else:  # Obstacles on the right
            # return left most point
            true_obs_point = max(obs_points, key=lambda x: x[1])
            obs_offset = [
                true_obs_point[0], true_obs_point[1] + .75]

        return ((true_obs_point[0], true_obs_point[1]), (obs_offset[0], obs_offset[1]))

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

    def keep_going(self, speed = 0.5, kind = "lane"):
        """Follows the lane based on camera data

        Args:
            speed (float, optional): The forward speed of the robot. Defaults to 0.5.
            kind (str, optional): The type of motion to use. Options are "lane" and "forward". Defaults to "lane".
        """
        cmd_vel = Twist()
        if kind == "forward":
            cmd_vel.linear.x = speed
            cmd_vel.angular.z = 0
            self.vel_cmd.publish(cmd_vel)
        elif kind == "lane":
            left = self.sliding_window(self.lanes, side="left")
            right = self.sliding_window(self.lanes, side="right")
            cmd_vel.linear.x = speed
            err = right - left
            cmd_vel.angular.z = np.clip(0.001 * err, -.05, .05)
            self.vel_cmd.publish(cmd_vel)
            if abs(err) < 1:
                self.store_current_heading()

    def odom_cb(self, msg):
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

    def scan_cb(self, msg):
        """Handles LaserScan messages

        Args:
            msg (LaserScan): LaserScan data
        """
        angle_range = msg.angle_max - msg.angle_min
        msg_len = len(msg.ranges)

        self.obstacles = []
        for i in range(len(msg.ranges)):
            angle = i * angle_range/msg_len - msg.angle_max
            dist = msg.ranges[i]
            if dist != np.inf:
                y_dist = dist * np.sin(angle)
                x_dist = dist * np.cos(angle)
                self.obstacles.append((x_dist, y_dist))
        self.scan_start = True

    def image_cb(self, msg):
        """Handles lane image messages

        Args:
            msg (Image): The Image data
        """
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        _, lanes = cv2.threshold(
            frame, 128, 255, cv2.THRESH_BINARY)
        self.lanes = lanes
        self.img_start = True

    def sliding_window(self, img, side = "left"):
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

    def average_point(self, *points):
        """Calculates the average of points

        Returns:
            tuple[float, float]: (x, y) representing the average of all the points
        """
        length = len(points)
        sum_x = reduce(lambda total, point: total + point[0], points, 0)
        sum_y = reduce(lambda total, point: total + point[1], points, 0)
        return (sum_x/length, sum_y/length)

    def find_shortest_path(self, start, end, path=[]):
        """Finds the shortest path between two nodes on the graph

        Args:
            start (str): The starting position or node
            end (str): The destination position or node
            path (list[str], optional): A list of previous path chosen. Defaults to [].

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
            if node.get_id() not in path:
                newpath = self.find_shortest_path(node.get_id(), end, path)
                if newpath:
                    if not shortest or len(newpath) < len(shortest):
                        shortest = newpath
        return shortest

    def nearest_node(self, p, nodes = []):
        """Takes a point and returns the node closest to that point

        Args:
            p (tuple[float, float]): The point
            nodes (list[Vertex], optional): A list of vertices to scan. If not provided, the entire graph is scanned. Defaults to []

        Returns:
            str: The node as a string
        """
        search_space = self.graph if len(nodes) == 0 else nodes
        prev_dist = np.inf
        for node in search_space:
            dist = self.distance(p, node.coordinates)
            if dist < prev_dist:
                nearest_node = node
                prev_dist = dist

        return nearest_node, prev_dist

    def distance(self, p1, p2):
        """Calculates the distance between two points

        Args:
            p1 (tuple[float, float]): The first point
            p2 (tuple[float, float]): The second point

        Returns:
            float: The scalar distance between the two points
        """
        return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def create_map(self):
        """Creates the graph intersection map of the PARC world

        Returns:
            Graph: The graph representation of the PARC world
        """
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
