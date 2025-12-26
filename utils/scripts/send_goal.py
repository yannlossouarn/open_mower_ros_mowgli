#!/usr/bin/env python3
import rospy
import actionlib
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

rospy.init_node("send_mbf_goal")

client = actionlib.SimpleActionClient("/move_base_flex/move_base", MoveBaseAction)
if not client.wait_for_server(rospy.Duration(10.0)):
    rospy.logerr("MBF MoveBase action server not available")
    raise SystemExit(1)

goal = MoveBaseGoal()
pose = PoseStamped()
pose.header.frame_id = "map"
pose.header.stamp = rospy.Time.now()
pose.pose.position.x = 1.5   # set your desired x
pose.pose.position.y = 0.0   # set your desired y
pose.pose.orientation.w = 1.0

goal.target_pose = pose
goal.controller = "FTCPlanner"   # <<-- set to your TEB controller name (or "FTCPlanner" if using that)

client.send_goal(goal)
finished = client.wait_for_result(rospy.Duration(60.0))
if not finished:
    rospy.logwarn("Goal not finished within timeout, cancelling")
    client.cancel_goal()
else:
    res = client.get_result()
    rospy.loginfo("Result: %s", str(res))
