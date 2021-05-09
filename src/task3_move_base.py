#!/usr/bin/env python

import sys

import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

try:
    # goal_x = float(sys.argv[1])
    # goal_y = float(sys.argv[2])
    goal_x = 0.0
    goal_y = -0.0
    goal_theta = -1.57079632679
except IndexError:
    rospy.logerr("usage: rosrun task2_node.py <goal_x> <goal_y>")
except ValueError as e:
    rospy.logfatal(f"{str(e)}")
    rospy.signal_shutdown("Fatal error")


def movebase_client():

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "odom"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = goal_x
    goal.target_pose.pose.position.y = goal_y
    goal.target_pose.pose.orientation.w = 0.1

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
