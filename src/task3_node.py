#!/usr/bin/env python
import sys

import actionlib
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseFeedback, MoveBaseGoal
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

try:
    goal_x = float(sys.argv[1])
    goal_y = float(sys.argv[2])
    goal_theta = float(sys.argv[3])
except IndexError:
    rospy.logerr("usage: rosrun task2_node.py <goal_x> <goal_y>")
    sys.exit(0)
except ValueError as e:
    rospy.logfatal(str(e))
    rospy.signal_shutdown("Fatal error: Could not parse goal")
    sys.exit(0)


class Task3:
    def __init__(self) -> None:
        rospy.init_node('task3_solution')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.vel_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.main()

    def main(self):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y
        goal.target_pose.pose.orientation.w = 0.1

        self.client.send_goal(goal, feedback_cb=self.feedback_cb)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            self.client.get_result()
        self.go_to_point((goal_x, goal_y), max_speed=0.5,)
        self.turn_to_heading(goal_theta)
        self.stop()

    def feedback_cb(self, msg):
        position_x = msg.base_position.pose.position.x
        position_y = msg.base_position.pose.position.y
        print(self.distance((position_x, position_y), (goal_x, goal_y)))
        if self.distance((position_x, position_y), (goal_x, goal_y)) < 2.5:  # Close enough
            self.client.cancel_all_goals()
            rospy.logerr("Taking manual control")

    def distance(self, p1, p2):
        return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def go_to_point(self, point, max_speed = 1, distance = 0.1):
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
            self.vel_cmd.publish(cmd_vel)
            rate.sleep()

    def turn_to_heading(self, heading):
        """Returns the robot to the set heading

        Args:
            heading (float): The heading to turn to
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = 0
        while abs(self.odom[2] - heading) > 0.01:
            err = (self.odom[2] - heading)
            cmd_vel.angular.z = -0.5 * err
            if rospy.is_shutdown():
                break
            self.vel_cmd.publish(cmd_vel)

    def stop(self):
        """Stops the robot
        """
        print("Stopping the robot")
        cmd_vel = Twist()
        cmd_vel.linear.x = 0
        cmd_vel.angular.z = 0
        self.vel_cmd.publish(cmd_vel)

    def odom_cb(self, msg):
        """Handles odometry messages

        Args:
            msg (Odometry): Odometry data
        """
        position = msg.pose.pose.position
        rotation = msg.pose.pose.orientation
        (_, _, theta) = euler_from_quaternion(
            [rotation.x, rotation.y, rotation.z, rotation.w])
        self.odom = (position.x, position.y, theta)


if __name__ == '__main__':
    try:
        Task3()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
