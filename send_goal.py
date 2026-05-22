#!/usr/bin/env python3
"""
Send a navigation goal [x, y] expressed in the robot's base_link frame
AT THE MOMENT of sending. The goal is transformed to the odom frame once
(snapshot), so the target position stays fixed even as the robot moves.

Usage:
    python3 send_goal.py              # sends [10, 0] ahead of robot right now
    python3 send_goal.py --x 5 --y 2  # custom offset in base_link at send time
"""

import argparse
import rospy
import tf
import actionlib
from geometry_msgs.msg import PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction


def send_goal(x, y):
    rospy.init_node('send_goal_node', anonymous=True)

    tf_listener = tf.TransformListener()
    rospy.loginfo("Waiting for tf (base_link -> odom)...")
    tf_listener.waitForTransform('odom', 'base_link', rospy.Time(0), rospy.Duration(5.0))

    # Build goal in base_link frame at this exact moment
    goal_in_base = PoseStamped()
    goal_in_base.header.frame_id = 'base_link'
    goal_in_base.header.stamp = rospy.Time(0)  # use latest available transform
    goal_in_base.pose.position.x = x
    goal_in_base.pose.position.y = y
    goal_in_base.pose.position.z = 0.0
    goal_in_base.pose.orientation = Quaternion(0, 0, 0, 1)

    # Transform to odom — snapshot at this instant, result is a fixed odom coordinate
    goal_in_odom = tf_listener.transformPose('odom', goal_in_base)
    rospy.loginfo("Goal in odom frame: x=%.2f, y=%.2f" % (
        goal_in_odom.pose.position.x, goal_in_odom.pose.position.y))

    # Publish to move_base_simple/goal for RViz visualization
    simple_goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)
    goal_in_odom.header.stamp = rospy.Time.now()
    rospy.sleep(0.5)  # wait for subscriber to connect
    simple_goal_pub.publish(goal_in_odom)
    rospy.loginfo("Published to /move_base_simple/goal for RViz.")

    # Send the fixed odom-frame goal to move_base
    nav_as = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    nav_as.wait_for_server()
    rospy.loginfo("Connected to move_base action server.")

    mb_goal = MoveBaseGoal()
    mb_goal.target_pose = goal_in_odom
    mb_goal.target_pose.header.stamp = rospy.Time.now()

    nav_as.send_goal(mb_goal)
    rospy.loginfo("Goal sent. Waiting for result...")
    nav_as.wait_for_result()

    state = nav_as.get_state()
    if state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached successfully!")
    else:
        rospy.logwarn("Goal failed with state: %d" % state)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Send a goal offset from robot pose at send time.')
    parser.add_argument('--x', type=float, default=7.0, help='Forward offset in base_link (default: 10.0)')
    parser.add_argument('--y', type=float, default=0.0,  help='Lateral offset in base_link (default: 0.0)')
    args = parser.parse_args()

    send_goal(args.x, args.y)
