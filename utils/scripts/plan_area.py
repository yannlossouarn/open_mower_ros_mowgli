#!/usr/bin/env python3
"""
Fetch a mowing area from the map service and submit it to the slic3r coverage planner.

Usage examples:
  # source ROS and workspace first
  source /opt/ros/$ROS_DISTRO/setup.bash
  source /home/yann/openmower/open_mower_ros_mowgli/devel/setup.bash

  # run with defaults (area_index=0)
  python3 utils/scripts/plan_area.py

  # override params via ROS param namespace
  python3 utils/scripts/plan_area.py _area_index:=1 _distance:=0.13

"""
import sys
import rospy
from mower_map.srv import GetMowingAreaSrv
from slic3r_coverage_planner.srv import PlanPath, PlanPathRequest


def main():
    rospy.init_node('plan_area_client', anonymous=True)

    # params (can be passed on command line as _param:=value)
    area_index = rospy.get_param('~area_index', rospy.get_param('area_index', 1))
    angle = float(rospy.get_param('~angle', rospy.get_param('angle', 180.0)))
    distance = float(rospy.get_param('~distance', rospy.get_param('distance', 0.17)))
    outer_offset = float(rospy.get_param('~outer_offset', rospy.get_param('outer_offset', 0.08)))
    outline_count = int(rospy.get_param('~outline_count', rospy.get_param('outline_count', 2)))
    outline_overlap_count = int(rospy.get_param('~outline_overlap_count', rospy.get_param('outline_overlap_count', 0)))
    fill_type = int(rospy.get_param('~fill_type', rospy.get_param('fill_type', 0)))

    map_service_name = rospy.get_param('~map_service', rospy.get_param('map_service', 'mower_map_service/get_mowing_area'))
    planner_service_name = rospy.get_param('~planner_service', rospy.get_param('planner_service', 'slic3r_coverage_planner/plan_path'))

    # If the configured service name is not present, try to auto-discover a matching service
    def find_service_containing(substr):
        try:
            services = rospy.get_service_list()
        except Exception:
            return None
        for s in services:
            if substr in s:
                return s
        return None

    # discover map service if default not found
    if not rospy.get_param('~map_service', None):
        found = find_service_containing('get_mowing_area')
        if found:
            map_service_name = found
    else:
        # if explicitly configured but not available, try to find similar
        if map_service_name not in rospy.get_service_list():
            found = find_service_containing('get_mowing_area')
            if found:
                rospy.logwarn('Configured map service %s not present, using %s', map_service_name, found)
                map_service_name = found

    # discover planner service if default not found
    if not rospy.get_param('~planner_service', None):
        found = find_service_containing('plan_path')
        if found:
            planner_service_name = found
    else:
        if planner_service_name not in rospy.get_service_list():
            found = find_service_containing('plan_path')
            if found:
                rospy.logwarn('Configured planner service %s not present, using %s', planner_service_name, found)
                planner_service_name = found

    rospy.loginfo('Waiting for services: %s and %s', map_service_name, planner_service_name)
    try:
        rospy.wait_for_service(map_service_name, timeout=30)
        rospy.wait_for_service(planner_service_name, timeout=30)
    except rospy.ROSException as e:
        rospy.logerr('Service not available: %s', e)
        return 2

    try:
        map_srv = rospy.ServiceProxy(map_service_name, GetMowingAreaSrv)
        plan_srv = rospy.ServiceProxy(planner_service_name, PlanPath)

        rospy.loginfo('Requesting area %d from %s', area_index, map_service_name)
        map_resp = map_srv(index=area_index)
    except rospy.ServiceException as e:
        rospy.logerr('Map service call failed: %s', e)
        return 3

    # build planner request
    req = PlanPathRequest()
    req.fill_type = fill_type
    req.angle = angle
    req.distance = distance
    req.outer_offset = outer_offset
    req.outline_count = outline_count
    req.outline_overlap_count = outline_overlap_count

    # copy outline and holes from map response
    req.outline = map_resp.area.area
    req.holes = list(map_resp.area.obstacles)

    rospy.loginfo('Calling planner %s for area %d (angle=%s, distance=%s)', planner_service_name, area_index, angle, distance)
    try:
        res = plan_srv(req)
    except rospy.ServiceException as e:
        rospy.logerr('Planner service call failed: %s', e)
        return 4

    rospy.loginfo('Planner returned %d paths', len(res.paths))
    for i, p in enumerate(res.paths):
        rospy.loginfo('Path %d has %d poses', i, len(p.path.poses))

    return 0


if __name__ == '__main__':
    sys.exit(main())
