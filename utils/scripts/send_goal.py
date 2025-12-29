#!/usr/bin/env python3
"""
send_goal.py

Tool to request a path plan from the mower's planner (MBF/FTCPlanner) between
two poses and publish the resulting nav_msgs/Path to `/planned_path` for
visualization. The start pose can be the current robot pose (looked up via TF)
or a manually provided pose.

Usage examples:
  ./send_goal.py --goal 1.0 0.0 0.0 --start-current
  ./send_goal.py --start 0.0 0.0 0.0 --goal 2.0 1.0 1.57

If MBF's GetPath action server isn't available the script will log an error.
"""

import argparse
import math
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
import tf


def make_pose_stamped(x, y, yaw, frame_id="map"):
    p = PoseStamped()
    p.header.frame_id = frame_id
    p.header.stamp = rospy.Time.now()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.position.z = 0.0
    q = tf.transformations.quaternion_from_euler(0, 0, float(yaw))
    p.pose.orientation.x = q[0]
    p.pose.orientation.y = q[1]
    p.pose.orientation.z = q[2]
    p.pose.orientation.w = q[3]
    return p


def lookup_current_pose(map_frame, base_frame, timeout=5.0):
    tfbuf = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfbuf)
    try:
        trans = tfbuf.lookup_transform(map_frame, base_frame, rospy.Time(0), rospy.Duration(timeout))
    except Exception as e:
        rospy.logerr("TF lookup failed: %s", e)
        return None
    ps = PoseStamped()
    ps.header = trans.header
    ps.header.frame_id = map_frame
    ps.pose.position.x = trans.transform.translation.x
    ps.pose.position.y = trans.transform.translation.y
    ps.pose.position.z = trans.transform.translation.z
    ps.pose.orientation = trans.transform.rotation
    return ps


def try_get_path_action(action_name):
    try:
        from mbf_msgs.msg import GetPathAction, GetPathGoal
    except Exception:
        rospy.logerr("mbf_msgs/GetPathAction not available in Python imports. Ensure move_base_flex/mbf_msgs is installed.")
        return None, None

    client = actionlib.SimpleActionClient(action_name, GetPathAction)
    rospy.loginfo("Waiting for GetPath action server '%s'...", action_name)
    if not client.wait_for_server(rospy.Duration(5.0)):
        rospy.logwarn("GetPath action server '%s' not available", action_name)
        return None, None

    return client, GetPathGoal


def main():
    parser = argparse.ArgumentParser(description="Request a planner-only path from the mower planner (MBF/FTCPlanner).")
    parser.add_argument("--start", nargs=3, type=float, metavar=("X","Y","YAW"), help="Start pose x y yaw (radians)")
    parser.add_argument("--start-current", action="store_true", help="Use the current robot pose as start (TF lookup)")
    parser.add_argument("--goal", nargs=3, required=True, type=float, metavar=("X","Y","YAW"), help="Goal pose x y yaw (radians)")
    parser.add_argument("--map-frame", default="map", help="Map frame to use (default: map)")
    parser.add_argument("--base-frame", default="base_link", help="Robot base frame for current pose lookup (default: base_link)")
    parser.add_argument("--action", default="/move_base_flex/get_path", help="GetPath action server name (default: /move_base_flex/get_path)")
    parser.add_argument("--controller", default="FTCPlanner", help="Planner/controller name to request (default: FTCPlanner)")
    parser.add_argument("--planner", default=None, help="MBF planner plugin name to request (e.g. GlobalPlanner)")
    parser.add_argument("--tolerance", default=0.0, type=float, help="Tolerance to pass to GetPath goal (default: 0.0)")
    parser.add_argument("--out", help="Optional output file to save path as a simple list of poses")
    args = parser.parse_args()

    rospy.init_node("planner_trace_path", anonymous=True)

    # Build start pose
    if args.start_current:
        start = lookup_current_pose(args.map_frame, args.base_frame)
        if start is None:
            rospy.logerr("Failed to get current robot pose from TF. Exiting.")
            return
    elif args.start:
        start = make_pose_stamped(args.start[0], args.start[1], args.start[2], frame_id=args.map_frame)
    else:
        rospy.logerr("Either --start or --start-current must be provided.")
        return

    goal = make_pose_stamped(args.goal[0], args.goal[1], args.goal[2], frame_id=args.map_frame)

    client, GetPathGoal = try_get_path_action(args.action)
    if client is None:
        rospy.logerr("No GetPath action available. Cannot request planner-only path.")
        return

    gp_goal = GetPathGoal()
    # MBF GetPathGoal uses 'use_start_pose', 'start_pose' and 'target_pose'
    try:
        gp_goal.use_start_pose = True
        gp_goal.start_pose = start
        gp_goal.target_pose = goal
    except Exception:
        # fallback to other possible field names
        if hasattr(gp_goal, 'use_start_pose'):
            try:
                gp_goal.use_start_pose = True
            except Exception:
                pass
        if hasattr(gp_goal, 'start_pose'):
            gp_goal.start_pose = start
        if hasattr(gp_goal, 'target_pose'):
            gp_goal.target_pose = goal

    # set planner and tolerance if available
    if hasattr(gp_goal, 'tolerance'):
        try:
            gp_goal.tolerance = float(args.tolerance)
        except Exception:
            pass

    if args.planner and hasattr(gp_goal, 'planner'):
        try:
            gp_goal.planner = args.planner
        except Exception:
            pass

    # also keep controller arg handling for backward compatibility
    if hasattr(gp_goal, 'planner') and not args.planner:
        try:
            gp_goal.planner = args.controller
        except Exception:
            pass

    rospy.loginfo("Sending GetPath goal to %s (start: %s -> goal: %s)", args.action, str(start.pose.position), str(goal.pose.position))
    client.send_goal(gp_goal)
    finished = client.wait_for_result(rospy.Duration(30.0))
    if not finished:
        rospy.logerr("GetPath action did not finish in time")
        client.cancel_goal()
        return

    result = client.get_result()
    if result is None:
        rospy.logerr("GetPath returned no result")
        return

    # Debug: log outcome and message
    try:
        outcome = result.result.outcome
        message = result.result.message
        rospy.loginfo("GetPath outcome: %s (%d)", message, outcome)
    except Exception:
        pass
    # Log full result for debugging
    try:
        rospy.loginfo("Full GetPath result: %s", str(result))
    except Exception:
        pass

    # If a path is present, log its size and first/last poses for inspection
    try:
        if hasattr(result.result, 'path') and result.result.path is not None:
            p = result.result.path
            rospy.loginfo("Result.path.header.frame_id=%s, poses=%d", getattr(p.header, 'frame_id', ''), len(p.poses))
            if len(p.poses) > 0:
                rospy.loginfo("First pose: %s", str(p.poses[0].pose.position))
                rospy.loginfo("Last pose: %s", str(p.poses[-1].pose.position))
    except Exception:
        pass

    # MBF GetPath result commonly contains a nav_msgs/Path in a field named 'path' or 'plan'
    path = None
    if hasattr(result, 'path'):
        path = result.path
    elif hasattr(result, 'plan'):
        path = result.plan
    elif hasattr(result, 'paths'):
        # sometimes planners return multiple paths
        paths = result.paths
        if isinstance(paths, (list, tuple)) and len(paths) > 0 and hasattr(paths[0], 'path'):
            path = paths[0].path

    if path is None:
        rospy.logerr("Could not extract nav_msgs/Path from GetPath result: %s", str(result))
        return

    # Publish the planned path for visualization
    pub = rospy.Publisher('/planned_path', Path, queue_size=1, latch=True)
    rospy.sleep(0.3)
    path.header.stamp = rospy.Time.now()
    pub.publish(path)
    rospy.loginfo("Published planned path with %d poses to /planned_path", len(path.poses))

    if args.out:
        try:
            with open(args.out, 'w') as f:
                for p in path.poses:
                    f.write(f"%f %f %f\n" % (p.pose.position.x, p.pose.position.y, tf.transformations.euler_from_quaternion([p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w])[2]))
            rospy.loginfo("Saved path to %s", args.out)
        except Exception as e:
            rospy.logwarn("Failed to save path: %s", e)


if __name__ == '__main__':
    main()
