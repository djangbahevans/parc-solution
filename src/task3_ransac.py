#! /usr/bin/env python
import sys
from math import atan2, cos, inf, pi, sin, sqrt
from typing import Tuple

import numpy as np
import rospy
from geometry_msgs.msg import PointStamped, Twist
from laser_line_extraction.msg import LineSegmentList
from nav_msgs.msg import Odometry
from tf.listener import TransformListener
from tf.transformations import euler_from_quaternion

# try:
#     goal_x = float(sys.argv[1])
#     goal_y = float(sys.argv[2])
# except IndexError:
#     rospy.logerr("usage: rosrun task2_node.py <goal_x> <goal_y>")
# except ValueError as e:
#     rospy.logfatal(f"{str(e)}")
#     rospy.signal_shutdown("Fatal error")

cmd_msg = Twist()
cmd_msg.linear.x = 0.5
start = 0
end = 0


class Line:
    def __init__(self, start, end) -> None:
        self.start = np.matrix(start).T
        self.end = np.matrix(end).T

    def angle(self) -> float:
        """Returns the angle of the line

        Returns:
            float: The angle of the line
        """
        return atan2(self.end[1, 0] - self.start[1, 0], self.end[0, 0] - self.start[0, 0])

    def connected(self, line) -> bool:
        """Checks if two lines share a common point

        Args:
            line (Line): A line to check
        """
        if (self.start[0, 0], self.start[1, 0]) == (line.start[0, 0], line.start[1, 0]):
            return True
        elif (self.start[0, 0], self.start[1, 0]) == (line.end[0, 0], line.end[1, 0]):
            return True
        elif (self.end[0, 0], self.end[1, 0]) == (line.start[0, 0], line.start[1, 0]):
            return True
        elif (self.end[0, 0], self.end[1, 0]) == (line.end[0, 0], line.end[1, 0]):
            return True

        return False

    def extend(self, point):
        start = (self.start[0, 0], self.start[1, 0])
        end = (self.end[0, 0], self.end[1, 0])
        if Line(start, point).length > Line(point, end).length:
            self.end = np.matrix(point).T
            kept = start
            other = end
        else:
            self.start = np.matrix(point).T
            kept = end
            other = start
        return kept, point, other

    def intersection(self, line):
        """Returns the intersection point of two lines

        Args:
            line (Line): The intersecting line

        Returns:
            (float, float): An x, y point of the point of intersection
        """
        # if (self.slope == inf and line.slope == inf) or abs(self.slope - line.slope) < 0.001:
        if (self.slope == inf and line.slope == inf):
            # print("Both lines vertical or parallel")
            return None

        if self.slope == inf:
            return line.slope * self.start[0, 0] + line.y_intercept

        if line.slope == inf:
            return self.slope * line.start[0, 0] + self.y_intercept

        x = (line.y_intercept - self.y_intercept)/(self.slope - line.slope)
        y = self.slope * x + self.y_intercept

        return (x, y)

    @property
    def length(self) -> float:
        """The length of the line

        Returns:
            float: The length of the line
        """
        return sqrt((self.start[0, 0] - self.end[0, 0])**2 + (self.start[1, 0] - self.end[1, 0])**2)

    def merge(self, line):
        point = self.intersection(line)
        if point is not None:
            self.extend(point)
            line.extend(point)

        if (self.start[0, 0], self.start[1, 0]) == (line.start[0, 0], line.start[1, 0]) or\
                (self.end[0, 0], self.end[1, 0]) == (line.end[0, 0], line.end[1, 0]):
            line.swap_direction()

    def offset(self, offset_distance, centroid=None):
        start_local = (0, 0)
        end_local = (self.length(), 0)
        theta = self.angle()

        offset_start_local = np.matrix(
            [[start_local[0]], [start_local[1] + offset_distance]])
        offset_end_local = np.matrix(
            [[end_local[0]], [end_local[1] + offset_distance]])

        tf_matrix = np.matrix([
            [cos(theta), sin(-theta)],
            [sin(theta), cos(theta)]
        ])

        start_global = tf_matrix * offset_start_local
        end_global = tf_matrix * offset_end_local

        start = np.matrix(
            (start_global[0, 0], start_global[1, 0])).T + self.start
        end = np.matrix((end_global[0, 0], end_global[1, 0])).T + self.start

        return Line((start[0, 0], start[1, 0]), (end[0, 0], end[1, 0]))

    @property
    def points(self):
        return ((self.start[0, 0], self.start[1, 0]), (self.end[0, 0], self.end[1, 0]))

    @property
    def slope(self):
        if self.start[0, 0] == self.end[0, 0]:
            return inf
        return (self.start[1, 0]-self.end[1, 0])/(self.start[0, 0]-self.end[0, 0])

    def swap_direction(self):
        temp = self.start
        self.start = self.end
        self.end = self.temp

    @property
    def y_intercept(self):
        return self.start[1, 0] - self.slope * self.start[0, 0]

    def __call__(self, *args, **kwds):
        return ((self.start[0, 0], self.start[1, 0]), (self.end[0, 0], self.end[1, 0]))

    def __repr__(self) -> str:
        return f"slope: {self.slope} \n intercept: {self.y_intercept}"


class Polyline:
    def __init__(self, lines) -> None:
        self.lines = lines
        points = set()
        for line in lines:
            points.update(line.points)

        self.points = list(points)
        print(self.points)
        self.sort_points()
        print(self.points)
        print()

    def add(self, line):
        self.lines.append(line)

    def centroid(self):
        x = list(map(lambda p: p[0], self.points))
        x_avg = sum(x)/len(x)
        y = list(map(lambda p: p[1], self.points))
        y_avg = sum(y)/len(y)

        return (x_avg, y_avg)

    def connect_lines(self):
        for i in range(len(self.lines) - 1):
            line1 = self.lines[i]
            line2 = self.lines[i+1]
            line1.merge(line2)

    def offset(self, offset_distance):
        offsets = []
        for line in self.lines:
            p_offset = line.offset(offset_distance)
            n_offset = line.offset(-offset_distance)
            offsets.append(line.offset(offset_distance))

        return Polyline(offsets)

    def sort_points(self, clockwise=True):
        centroid = self.centroid()

        def angle(p, c):
            ang = atan2(p[1]-c[1], p[0]-c[0])
            ang = (ang if ang > 0 else (2*pi + ang)) * 360 / (2*pi)
            return ang

        self.points.sort(key=lambda p: angle(p, centroid))

    def __call__(self, *args, **kwds):
        return list(map(lambda line: line(), self.lines))


class Main:
    def __init__(self) -> None:
        rospy.init_node("task3_solution")

        self.take_snapshot = True
        self.lines = []
        self.position = None

        self.listener = TransformListener()
        self.listener.waitForTransform(
            "odom", "camera_link", rospy.Time(), rospy.Duration(20))

        vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        sg_sub = rospy.Subscriber(
            "/line_segments", LineSegmentList, self.line_segmentCb)
        odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        rospy.spin()

        # self.run()

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.take_snapshot == False:
                if len(self.lines) == 0:
                    self.take_snapshot = True
                    continue

            rate.sleep()

    def odom_cb(self, odom):
        pose = odom.pose.pose
        rotation = odom.pose.pose.orientation
        (_, _, theta) = euler_from_quaternion(
            [rotation.x, rotation.y, rotation.z, rotation.w])

        self.position = Twist()
        self.position.linear.x = pose.position.x
        self.position.linear.y = pose.position.y
        self.position.angular.z = theta

    def line_segmentCb(self, segments_data):
        self.lines = []
        take_snapshot = self.take_snapshot
        for line in segments_data.line_segments:
            start = PointStamped()
            start.header.frame_id = segments_data.header.frame_id
            start.header.stamp = rospy.Time(0)
            start.point.x = line.start[0]
            start.point.y = line.start[1]
            start.point.z = 0.0
            _start = self.listener.transformPoint("odom", start)

            end = PointStamped()
            end.header.frame_id = segments_data.header.frame_id
            end.header.stamp = rospy.Time(0)
            end.point.x = line.end[0]
            end.point.y = line.end[1]
            end.point.z = 0.0
            _end = self.listener.transformPoint("odom", end)

            if take_snapshot:
                self.lines.append(Line((_start.point.x, _start.point.y),
                                       (_end.point.x, _end.point.y)))

            # self.take_snapshot = False

        Polyline(self.lines)


if __name__ == "__main__":
    # x = Line((8.8, 3.2), (6.0, 1.1))
    # y = Line((8.8, 1.2), (4.1, 1.1))
    # print(x.merged(y))
    # Line.merge(x, y)
    # print(x.merged(y))
    try:
        Main()
    except rospy.ROSInterruptException:
        sys.exit(0)
