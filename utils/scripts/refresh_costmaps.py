#!/usr/bin/env python3
"""
refresh_costmaps.py

Watch YAML files and refresh costmap-related parameters
by loading them into the ROS parameter server and calling
clear-costmap services (default: /move_base_flex/clear_costmaps).

This script polls provided glob paths for mtime changes and
on change runs `rosparam load <file>` (via subprocess) and
calls configured services (std_srvs/Empty).

Usage examples:
  ./refresh_costmaps.py --paths "config/*costmap*.yaml" --interval 2.0
  ./refresh_costmaps.py --paths "config/*costmap*.yaml" --services "/move_base_flex/clear_costmaps,/local_costmap/clear" --once
"""

import argparse
import glob
import os
import time
import subprocess
import rospy
from std_srvs.srv import Empty


def find_files(patterns):
    files = []
    for p in patterns:
        files.extend(sorted(glob.glob(p, recursive=True)))
    # remove duplicates while preserving order
    seen = set()
    out = []
    for f in files:
        if f not in seen:
            seen.add(f)
            out.append(f)
    return out


def mtimes(files):
    d = {}
    for f in files:
        try:
            d[f] = os.path.getmtime(f)
        except Exception:
            d[f] = None
    return d


def load_rosparam(file_path):
    rospy.loginfo("Loading rosparam from %s", file_path)
    try:
        subprocess.run(["rosparam", "load", file_path], check=True)
        rospy.loginfo("rosparam loaded: %s", file_path)
    except Exception as e:
        rospy.logwarn("rosparam load failed for %s: %s", file_path, e)


def call_clear_services(services, timeout=5.0):
    for svc in services:
        try:
            rospy.wait_for_service(svc, timeout=timeout)
            proxy = rospy.ServiceProxy(svc, Empty)
            proxy()
            rospy.loginfo("Called service %s", svc)
        except Exception as e:
            rospy.logwarn("Failed to call service %s: %s", svc, e)


def main():
    parser = argparse.ArgumentParser(description="Watch YAML files and reload rosparams + clear costmaps on change.")
    parser.add_argument("--paths", nargs='+', default=["**/local_costmap*.yaml", "**/global_costmap*.yaml", "**/costmap_common*.yaml"], help="Glob patterns of YAML files to watch (default: **/*.yaml)")
    parser.add_argument("--interval", type=float, default=1.0, help="Polling interval in seconds (default: 1.0)")
    parser.add_argument("--services", default="/move_base_flex/clear_costmaps", help="Comma-separated list of services to call after reload (default: /move_base_flex/clear_costmaps)")
    parser.add_argument("--once", action='store_true', help="Run an initial load and exit instead of watching continuously")
    parser.add_argument("--no-initial", action='store_true', help="Do not perform an initial load; only react to subsequent changes")
    args = parser.parse_args()

    rospy.init_node('refresh_costmaps', anonymous=True)

    patterns = args.paths
    services = [s.strip() for s in args.services.split(',') if s.strip()]

    rospy.loginfo("CWD: %s", os.getcwd())
    rospy.loginfo("Watching patterns: %s", patterns)
    rospy.loginfo("Will call services: %s", services)

    files = find_files(patterns)
    if not files:
        rospy.logwarn("No files found for patterns %s (try running from repo root or pass --paths)", patterns)

    prev = mtimes(files)

    if not args.no_initial:
        # initial load of all found files
        for f in files:
            load_rosparam(f)
        if services:
            call_clear_services(services)

    if args.once:
        rospy.loginfo("--once specified, exiting after initial load")
        return

    try:
        while not rospy.is_shutdown():
            files = find_files(patterns)
            cur = mtimes(files)

            # detect new or changed files
            changed = []
            for f in cur:
                if f not in prev or prev.get(f) != cur.get(f):
                    changed.append(f)

            if changed:
                rospy.loginfo("Detected changes in %d files", len(changed))
                for f in changed:
                    load_rosparam(f)
                if services:
                    call_clear_services(services)

            prev = cur
            time.sleep(max(0.01, float(args.interval)))
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
