// Created by Clemens Elflein on 2/21/22.
// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
//
// This file is part of OpenMower.
//
// OpenMower is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, version 3 of the License.
//
// OpenMower is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with OpenMower. If not, see
// <https://www.gnu.org/licenses/>.
//
#include "MowingBehavior.h"

#include <cryptopp/cryptlib.h>
#include <cryptopp/hex.h>
#include <cryptopp/sha.h>
#include <nav_msgs/Path.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>

#include "mower_logic/CheckPoint.h"
#include "mower_map/ClearNavPointSrv.h"
#include "mower_map/GetMowingAreaSrv.h"
#include "mower_map/SetNavPointSrv.h"
#include "xbot_msgs/AbsolutePose.h"

extern ros::ServiceClient mapClient;
extern ros::ServiceClient pathClient;
extern ros::ServiceClient pathProgressClient;
extern ros::ServiceClient setNavPointClient;
extern ros::ServiceClient clearNavPointClient;

extern actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>* mbfClient;
extern actionlib::SimpleActionClient<mbf_msgs::ExePathAction>* mbfClientExePath;
extern mower_logic::MowerLogicConfig getConfig();
extern void setConfig(mower_logic::MowerLogicConfig);
extern xbot_msgs::AbsolutePose getPose();

extern void registerActions(std::string prefix, const std::vector<xbot_msgs::ActionInfo>& actions);

MowingBehavior MowingBehavior::INSTANCE;

std::string MowingBehavior::state_name() {
  if (paused) {
    return "PAUSED";
  }
  return "MOWING";
}

Behavior* MowingBehavior::execute() {
  shared_state->active_semiautomatic_task = true;

  ROS_INFO_STREAM("MowingBehavior: execute() - starting from area="
                  << currentMowingArea << " path=" << currentMowingPath << " pathIndex=" << currentMowingPathIndex
                  << " (from checkpoint if any)");

  while (ros::ok() && !aborted) {
    if (currentMowingPaths.empty() && !create_mowing_plan(currentMowingArea)) {
      ROS_INFO_STREAM("MowingBehavior: Could not create mowing plan, docking");
      // Start again from first area next time.
      reset();
      // We cannot create a plan, so we're probably done. Go to docking station
      return &DockingBehavior::INSTANCE;
    }

    // No plan will be created if the area is skipped
    if (currentMowingPaths.empty()) {
      currentMowingArea++;
      currentMowingPath = 0;
      currentMowingPathIndex = 0;
      continue;
    }

    // We have a plan, execute it
    ROS_INFO_STREAM("MowingBehavior: Executing mowing plan");
    bool finished = execute_mowing_plan();
    if (finished) {
      // skip to next area if current
      ROS_INFO_STREAM("MowingBehavior: Executing mowing plan - finished");
      currentMowingArea++;
      currentMowingPaths.clear();
      currentMowingPath = 0;
      currentMowingPathIndex = 0;
    }
  }

  if (!ros::ok()) {
    // something went wrong
    return nullptr;
  }
  // we got aborted, go to docking station
  return &DockingBehavior::INSTANCE;
}

void MowingBehavior::enter() {
  skip_area = false;
  skip_path = false;
  paused = aborted = false;

  for (auto& a : actions) {
    a.enabled = true;
  }
  registerActions("mower_logic:mowing", actions);
}

void MowingBehavior::exit() {
  for (auto& a : actions) {
    a.enabled = false;
  }
  registerActions("mower_logic:mowing", actions);
}

void MowingBehavior::reset() {
  currentMowingPaths.clear();
  currentMowingArea = 0;
  currentMowingPath = 0;
  currentMowingPathIndex = 0;
  // increase cumulative mowing angle offset increment
  currentMowingAngleIncrementSum = std::fmod(currentMowingAngleIncrementSum + getConfig().mow_angle_increment, 360);
  checkpoint();

  if (config.automatic_mode == eAutoMode::SEMIAUTO) {
    ROS_INFO_STREAM("MowingBehavior: Finished semiautomatic task");
    shared_state->active_semiautomatic_task = false;
  }
}

bool MowingBehavior::needs_gps() {
  return true;
}

bool MowingBehavior::mower_enabled() {
  return mowerEnabled;
}

void MowingBehavior::update_actions() {
  for (auto& a : actions) {
    a.enabled = true;
  }

  // pause / resume switch. other actions are always available
  actions[0].enabled = !(requested_pause_flag & pauseType::PAUSE_MANUAL);
  actions[1].enabled = requested_pause_flag & pauseType::PAUSE_MANUAL;

  registerActions("mower_logic:mowing", actions);
}

bool MowingBehavior::create_mowing_plan(int area_index) {
  ROS_INFO_STREAM("MowingBehavior: Creating mowing plan for area: " << area_index);
  // Delete old plan and progress.
  currentMowingPaths.clear();

  // get the mowing area
  mower_map::GetMowingAreaSrv mapSrv;
  mapSrv.request.index = area_index;
  if (!mapClient.call(mapSrv)) {
    ROS_ERROR_STREAM("MowingBehavior: Error loading mowing area");
    return false;
  }

  if (!mapSrv.response.area.active) {
    ROS_INFO_STREAM("MowingBehavior: Skipping inactive mowing area");
    return true;
  }

  // Area orientation is the same as the first point, unless explicitly specified in the area attributes
  double angle = 0;
  if (!std::isnan(mapSrv.response.area.angle)) {
    angle = mapSrv.response.area.angle;
    ROS_INFO_STREAM("MowingBehavior: Using explicitly specified mow angle: " << angle);
  } else {
    auto points = mapSrv.response.area.area.points;
    if (points.size() >= 2) {
      tf2::Vector3 first(points[0].x, points[0].y, 0);
      for (auto point : points) {
        tf2::Vector3 second(point.x, point.y, 0);
        auto diff = second - first;
        if (diff.length() > 2.0) {
          // we have found a point that has a distance of > 2 m, calculate the angle
          angle = atan2(diff.y(), diff.x());
          ROS_INFO_STREAM("MowingBehavior: Detected mow angle: " << angle);
          break;
        }
      }
    }
  }

  // add mowing angle offset increment and return into the <-180, 180> range
  double mow_angle_offset = std::fmod(getConfig().mow_angle_offset + currentMowingAngleIncrementSum + 180, 360);
  if (mow_angle_offset < 0) mow_angle_offset += 360;
  mow_angle_offset -= 180;
  ROS_INFO_STREAM("MowingBehavior: mowing angle offset (deg): " << mow_angle_offset);
  if (config.mow_angle_offset_is_absolute) {
    angle = mow_angle_offset * (M_PI / 180.0);
    ROS_INFO_STREAM("MowingBehavior: Custom mowing angle: " << angle);
  } else {
    angle = angle + mow_angle_offset * (M_PI / 180.0);
    ROS_INFO_STREAM("MowingBehavior: Auto-detected mowing angle + mowing angle offset: " << angle);
  }

  // calculate coverage
  const auto& area = mapSrv.response.area;
  auto overrideOrGlobal = [](auto override, auto global, auto sentinel) {
    return (override != sentinel) ? override : global;
  };

  slic3r_coverage_planner::PlanPath pathSrv;
  pathSrv.request.angle = angle;
  pathSrv.request.outline_count = overrideOrGlobal(area.outline_count, config.outline_count, -1);
  pathSrv.request.outline_overlap_count =
      overrideOrGlobal(area.outline_overlap_count, config.outline_overlap_count, -1);
  pathSrv.request.outline = area.area;
  pathSrv.request.holes = area.obstacles;
  pathSrv.request.fill_type = slic3r_coverage_planner::PlanPathRequest::FILL_LINEAR;
  pathSrv.request.outer_offset = std::isnan(area.outline_offset) ? config.outline_offset : area.outline_offset;
  pathSrv.request.distance = config.tool_width;
  if (!pathClient.call(pathSrv)) {
    ROS_ERROR_STREAM("MowingBehavior: Error during coverage planning");
    return false;
  }

  currentMowingPaths = pathSrv.response.paths;

  // Calculate mowing plan digest from the poses
  // TODO: move to slic3r_coverage_planner
  CryptoPP::SHA256 hash;
  byte digest[CryptoPP::SHA256::DIGESTSIZE];
  for (const auto& path : currentMowingPaths) {
    for (const auto& pose_stamped : path.path.poses) {
      hash.Update(reinterpret_cast<const byte*>(&pose_stamped.pose), sizeof(geometry_msgs::Pose));
    }
  }
  hash.Final((byte*)&digest[0]);
  CryptoPP::HexEncoder encoder;
  std::string mowingPlanDigest = "";
  encoder.Attach(new CryptoPP::StringSink(mowingPlanDigest));
  encoder.Put(digest, sizeof(digest));
  encoder.MessageEnd();

  // Proceed to checkpoint?
  if (mowingPlanDigest == currentMowingPlanDigest) {
    ROS_INFO_STREAM("MowingBehavior: Advancing to checkpoint, path: " << currentMowingPath
                                                                      << " index: " << currentMowingPathIndex);
  } else {
    ROS_INFO_STREAM("MowingBehavior: Ignoring checkpoint for plan ("
                    << currentMowingPlanDigest << ") current mowing plan is (" << mowingPlanDigest << ")");
    // Plan has changed so must restart the area
    currentMowingPlanDigest = mowingPlanDigest;
    currentMowingPath = 0;
    currentMowingPathIndex = 0;
  }

  return true;
}

int getCurrentMowPathIndex() {
  ftc_local_planner::PlannerGetProgress progressSrv;
  int currentIndex = -1;
  if (pathProgressClient.call(progressSrv)) {
    currentIndex = progressSrv.response.index;
  } else {
    ROS_ERROR("MowingBehavior: getMowIndex() - Error getting progress from FTC planner");
  }
  return (currentIndex);
}

void printNavState(int state) {
  switch (state) {
    case actionlib::SimpleClientGoalState::PENDING: ROS_INFO(">>> State: Pending <<<"); break;
    case actionlib::SimpleClientGoalState::ACTIVE: ROS_INFO(">>> State: Active <<<"); break;
    case actionlib::SimpleClientGoalState::RECALLED: ROS_INFO(">>> State: Recalled <<<"); break;
    case actionlib::SimpleClientGoalState::REJECTED: ROS_INFO(">>> State: Rejected <<<"); break;
    case actionlib::SimpleClientGoalState::PREEMPTED: ROS_INFO(">>> State: Preempted <<<"); break;
    case actionlib::SimpleClientGoalState::ABORTED: ROS_INFO(">>> State: Aborted <<<"); break;
    case actionlib::SimpleClientGoalState::SUCCEEDED: ROS_INFO(">>> State: Succeeded <<<"); break;
    case actionlib::SimpleClientGoalState::LOST: ROS_INFO(">>> State: Lost <<<"); break;
    default: ROS_INFO(">>> State: Unknown Hu ? <<<"); break;
  }
}

bool MowingBehavior::execute_mowing_plan() {
  int first_point_attempt_counter = 0;
  int first_point_trim_counter = 0;
  ros::Time paused_time(0.0);

  // loop through all mowingPaths to execute the plan fully.
  while (currentMowingPath < currentMowingPaths.size() && ros::ok() && !aborted) {
    ////////////////////////////////////////////////
    // PAUSE HANDLING
    ////////////////////////////////////////////////
    if (requested_pause_flag) {  // pause was requested
      paused = true;
      mowerEnabled = false;
      u_int8_t last_requested_pause_flags = 0;
      while (requested_pause_flag && !aborted)  // while emergency and/or manual pause not asked to continue, we wait
      {
        if (last_requested_pause_flags != requested_pause_flag) {
          update_actions();
        }
        last_requested_pause_flags = requested_pause_flag;

        std::string pause_reason = "";
        if (requested_pause_flag & pauseType::PAUSE_EMERGENCY) {
          pause_reason += "on EMERGENCY";
          if (requested_pause_flag & pauseType::PAUSE_MANUAL) {
            pause_reason += " and ";
          }
        }
        if (requested_pause_flag & pauseType::PAUSE_MANUAL) {
          pause_reason += "waiting for CONTINUE";
        }
        ROS_INFO_STREAM_THROTTLE(30, "MowingBehavior: PAUSED (" << pause_reason << ")");
        ros::Rate r(1.0);
        r.sleep();
      }
      // we will drop into paused, thus will also wait for GPS to be valid again
    }
    if (paused) {
      paused_time = ros::Time::now();
      while (!this->hasGoodGPS() && !aborted)  // while no good GPS we wait
      {
        ROS_INFO_STREAM("MowingBehavior: PAUSED (" << (ros::Time::now() - paused_time).toSec()
                                                   << "s) (waiting for GPS)");
        ros::Rate r(1.0);
        r.sleep();
      }
      ROS_INFO_STREAM("MowingBehavior: CONTINUING");
      paused = false;
      update_actions();
    }

    auto& path = currentMowingPaths[currentMowingPath];
    ROS_INFO_STREAM("MowingBehavior: Path segment length: " << path.path.poses.size() << " poses.");

    // Check if path is empty. If so, directly skip it
    if (currentMowingPathIndex >= path.path.poses.size()) {
      ROS_INFO_STREAM("MowingBehavior: Skipping empty path.");
      currentMowingPath++;
      currentMowingPathIndex = 0;
      continue;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    // DRIVE TO THE FIRST POINT OF THE MOW PATH
    //
    // Three strategies (tried in order, falling back to the next if the current one fails):
    //
    //  2b) use_teb_for_transition=true  → single MoveBaseGoal with "TransitionPlanner" (TEB).
    //      TEB generates a footprint-aware curved trajectory that arrives with the correct
    //      orientation; no in-place rotation near obstacles.
    //
    //  2a) approach_enabled=true (default, FTC-based)
    //      → navigate to an approach waypoint (approach_distance m behind start along the mowing
    //        direction) with MoveBase, then execute a short ExePath to the actual start.
    //        The robot arrives at start already aligned → POST_ROTATE at start ≈ 0°.
    //
    //  fallback) standard single MoveBaseGoal to start point with FTCPlanner (original behaviour).
    //
    // After max_first_point_attempts failures the retreat recovery (Phase 3) is tried once:
    // the robot navigates back to the start of the previous path to escape the obstacle zone.
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    {
      ROS_INFO_STREAM("MowingBehavior: (FIRST POINT)  Moving to path segment starting point");
      if (path.is_outline && getConfig().add_fake_obstacle) {
        mower_map::SetNavPointSrv set_nav_point_srv;
        set_nav_point_srv.request.nav_pose = path.path.poses[currentMowingPathIndex].pose;
        setNavPointClient.call(set_nav_point_srv);
        sleep(1);
      }

      const auto localConfig = getConfig();
      const auto& startPose = path.path.poses[currentMowingPathIndex];
      bool first_point_reached = false;

      // -----------------------------------------------------------------------
      // Helper lambda: run the MBF MoveBase wait-loop.
      // Handles skip_area / skip_path / abort / pause the same way as before.
      // Sets `reached` to true on SUCCEEDED; returns false if we must immediately
      // return from execute_mowing_plan().
      // -----------------------------------------------------------------------
      // (Implemented inline for both approach and fallback paths below.)

      // -----------------------------------------------------------------------
      // Phase 2b: TEB transition planner
      // Navigate to the approach waypoint (approach_distance m behind strip
      // start) with TEB so the robot arrives already aligned and clear of
      // obstacles.  Phase 2a then does the final short glide-in with FTC.
      // Sending TEB to the strip start itself is infeasible when the start is
      // obstacle-adjacent (TEB's min_obstacle_dist prevents it from reaching
      // that point).
      // -----------------------------------------------------------------------
      if (localConfig.use_teb_for_transition && localConfig.approach_enabled) {
        tf2::Quaternion q_start(startPose.pose.orientation.x, startPose.pose.orientation.y,
                                startPose.pose.orientation.z, startPose.pose.orientation.w);
        double mow_yaw = 2.0 * std::atan2(q_start.z(), q_start.w());

        geometry_msgs::PoseStamped approachPose;
        approachPose.header = startPose.header;
        approachPose.pose.position.x = startPose.pose.position.x - localConfig.approach_distance * std::cos(mow_yaw);
        approachPose.pose.position.y = startPose.pose.position.y - localConfig.approach_distance * std::sin(mow_yaw);
        approachPose.pose.position.z = 0.0;
        approachPose.pose.orientation = startPose.pose.orientation;

        auto robotPose = getPose();
        double dx = approachPose.pose.position.x - robotPose.pose.pose.position.x;
        double dy = approachPose.pose.position.y - robotPose.pose.pose.position.y;
        double dist_to_approach = std::sqrt(dx * dx + dy * dy);

        if (dist_to_approach > localConfig.approach_min_distance) {
          ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) Using TEB TransitionPlanner to approach waypoint (dist="
                          << dist_to_approach << "m).");
          mbf_msgs::MoveBaseGoal mbGoal;
          mbGoal.target_pose = approachPose;
          mbGoal.controller = "TransitionPlanner";
          mbfClient->sendGoal(mbGoal);
          sleep(1);
          ros::Rate r_teb(10);
          while (ros::ok()) {
            auto st = mbfClient->getState();
            if (st.state_ == actionlib::SimpleClientGoalState::ACTIVE ||
                st.state_ == actionlib::SimpleClientGoalState::PENDING) {
              if (skip_area) {
                mbfClient->cancelAllGoals();
                mowerEnabled = false;
                currentMowingPaths.clear();
                skip_area = false;
                return true;
              }
              if (skip_path) {
                skip_path = false;
                currentMowingPath++;
                currentMowingPathIndex = 0;
                return false;
              }
              if (aborted) {
                mbfClient->cancelAllGoals();
                mowerEnabled = false;
                return false;
              }
              if (requested_pause_flag) {
                mbfClient->cancelAllGoals();
                mowerEnabled = false;
                return false;
              }
            } else {
              break;
            }
            r_teb.sleep();
          }
          if (mbfClient->getState().state_ == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) TEB TransitionPlanner reached approach waypoint.");
          } else {
            ROS_WARN_STREAM("MowingBehavior: (FIRST POINT) TEB TransitionPlanner failed (state="
                            << mbfClient->getState().state_ << "), proceeding to FTC approach.");
          }
        } else {
          ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) Already near approach waypoint (dist="
                          << dist_to_approach << "m), skipping TEB transit.");
        }
      }

      // -----------------------------------------------------------------------
      // Phase 2a: approach-waypoint strategy (FTC-based)
      // Runs after TEB transit (use_teb_for_transition=true) or standalone.
      // -----------------------------------------------------------------------
      if (!first_point_reached && localConfig.approach_enabled) {
        // Extract mowing direction from the start pose orientation.
        tf2::Quaternion q_start(startPose.pose.orientation.x, startPose.pose.orientation.y,
                                startPose.pose.orientation.z, startPose.pose.orientation.w);
        double mow_yaw = 2.0 * std::atan2(q_start.z(), q_start.w());

        // Place the approach waypoint approach_distance m behind start along the mowing direction.
        geometry_msgs::PoseStamped approachPose;
        approachPose.header = startPose.header;
        approachPose.pose.position.x = startPose.pose.position.x - localConfig.approach_distance * std::cos(mow_yaw);
        approachPose.pose.position.y = startPose.pose.position.y - localConfig.approach_distance * std::sin(mow_yaw);
        approachPose.pose.position.z = 0.0;
        approachPose.pose.orientation = startPose.pose.orientation;  // same heading as mowing strip

        // Midpoint between approach and start (needed so ExePath has ≥ 3 poses for FTC).
        geometry_msgs::PoseStamped midPose;
        midPose.header = startPose.header;
        midPose.pose.position.x = (approachPose.pose.position.x + startPose.pose.position.x) * 0.5;
        midPose.pose.position.y = (approachPose.pose.position.y + startPose.pose.position.y) * 0.5;
        midPose.pose.position.z = 0.0;
        midPose.pose.orientation = startPose.pose.orientation;

        // Skip the approach if the robot is already closer than approach_min_distance to it.
        auto robotPose = getPose();
        double dx = approachPose.pose.position.x - robotPose.pose.pose.position.x;
        double dy = approachPose.pose.position.y - robotPose.pose.pose.position.y;
        double dist_to_approach = std::sqrt(dx * dx + dy * dy);

        if (dist_to_approach > localConfig.approach_min_distance) {
          ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) Approach waypoint at dist="
                          << dist_to_approach << "m, mow_yaw=" << (mow_yaw * 180.0 / M_PI) << "deg.");

          // -- Step 1: MoveBase to approach waypoint --------------------------------
          mbf_msgs::MoveBaseGoal mbGoal;
          mbGoal.target_pose = approachPose;
          mbGoal.controller = "FTCPlanner";
          mbfClient->sendGoal(mbGoal);
          sleep(1);
          ros::Rate r_ap(10);
          bool approach_ok = false;
          while (ros::ok()) {
            auto st = mbfClient->getState();
            if (st.state_ == actionlib::SimpleClientGoalState::ACTIVE ||
                st.state_ == actionlib::SimpleClientGoalState::PENDING) {
              if (skip_area) {
                mbfClient->cancelAllGoals();
                mowerEnabled = false;
                currentMowingPaths.clear();
                skip_area = false;
                return true;
              }
              if (skip_path) {
                skip_path = false;
                currentMowingPath++;
                currentMowingPathIndex = 0;
                return false;
              }
              if (aborted) {
                mbfClient->cancelAllGoals();
                mowerEnabled = false;
                return false;
              }
              if (requested_pause_flag) {
                mbfClient->cancelAllGoals();
                mowerEnabled = false;
                return false;
              }
            } else {
              break;
            }
            r_ap.sleep();
          }
          approach_ok = (mbfClient->getState().state_ == actionlib::SimpleClientGoalState::SUCCEEDED);

          if (approach_ok) {
            // -- Step 2: ExePath [approach → mid → start] to align with mowing direction --
            nav_msgs::Path align_path;
            align_path.header = startPose.header;
            align_path.poses = {approachPose, midPose, startPose};

            mbf_msgs::ExePathGoal exeGoal;
            exeGoal.path = align_path;
            exeGoal.angle_tolerance = 0.1;  // ~6°
            exeGoal.dist_tolerance = 0.15;
            exeGoal.tolerance_from_action = true;
            exeGoal.controller = "FTCPlanner";
            mbfClientExePath->sendGoal(exeGoal);
            sleep(1);
            ros::Rate r_exe(10);
            while (ros::ok()) {
              auto st = mbfClientExePath->getState();
              if (st.state_ == actionlib::SimpleClientGoalState::ACTIVE ||
                  st.state_ == actionlib::SimpleClientGoalState::PENDING) {
                if (skip_area) {
                  mbfClientExePath->cancelAllGoals();
                  mowerEnabled = false;
                  currentMowingPaths.clear();
                  skip_area = false;
                  return true;
                }
                if (skip_path) {
                  skip_path = false;
                  currentMowingPath++;
                  currentMowingPathIndex = 0;
                  return false;
                }
                if (aborted) {
                  mbfClientExePath->cancelAllGoals();
                  mowerEnabled = false;
                  return false;
                }
                if (requested_pause_flag) {
                  mbfClientExePath->cancelAllGoals();
                  mowerEnabled = false;
                  return false;
                }
              } else {
                break;
              }
              r_exe.sleep();
            }
            auto exe_state = mbfClientExePath->getState().state_;
            if (exe_state == actionlib::SimpleClientGoalState::SUCCEEDED ||
                exe_state == actionlib::SimpleClientGoalState::PREEMPTED) {
              first_point_reached = true;
              ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) Approach path succeeded.");
            } else {
              ROS_WARN_STREAM("MowingBehavior: (FIRST POINT) Approach ExePath failed (state="
                              << exe_state << "), falling back to standard navigation.");
            }
          } else {
            ROS_WARN_STREAM("MowingBehavior: (FIRST POINT) Approach MoveBase failed (state="
                            << mbfClient->getState().state_ << "), falling back to standard navigation.");
          }
        } else {
          ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) Robot already close to approach point ("
                          << dist_to_approach << "m), skipping approach waypoint.");
        }
      }

      // -----------------------------------------------------------------------
      // Fallback: original MoveBase to start point with FTCPlanner
      // -----------------------------------------------------------------------
      actionlib::SimpleClientGoalState current_status(actionlib::SimpleClientGoalState::PENDING);
      if (!first_point_reached) {
        mbf_msgs::MoveBaseGoal moveBaseGoal;
        moveBaseGoal.target_pose = startPose;
        moveBaseGoal.controller = "FTCPlanner";
        mbfClient->sendGoal(moveBaseGoal);
        sleep(1);
        ros::Rate r(10);

        // wait for path execution to finish
        while (ros::ok()) {
          current_status = mbfClient->getState();
          if (current_status.state_ == actionlib::SimpleClientGoalState::ACTIVE ||
              current_status.state_ == actionlib::SimpleClientGoalState::PENDING) {
            // path is being executed, everything seems fine.
            // check if we should pause or abort mowing
            if (skip_area) {
              ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) SKIP AREA was requested.");
              // remove all paths in current area and return true
              mowerEnabled = false;
              mbfClientExePath->cancelAllGoals();
              currentMowingPaths.clear();
              skip_area = false;
              return true;
            }
            if (skip_path) {
              skip_path = false;
              currentMowingPath++;
              currentMowingPathIndex = 0;
              return false;
            }
            if (aborted) {
              ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) ABORT was requested - stopping path execution.");
              mbfClientExePath->cancelAllGoals();
              mowerEnabled = false;
              return false;
            }
            if (requested_pause_flag) {
              ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) PAUSE was requested - stopping path execution.");
              mbfClientExePath->cancelAllGoals();
              mowerEnabled = false;
              return false;
            }
          } else {
            ROS_INFO_STREAM("MowingBehavior: (FIRST POINT)  Got status "
                            << current_status.state_ << " from MBF/FTCPlanner -> Stopping path execution.");
            // we're done, break out of the loop
            break;
          }
          r.sleep();
        }

        if (current_status.state_ == actionlib::SimpleClientGoalState::SUCCEEDED) {
          first_point_reached = true;
        }
      }

      // -----------------------------------------------------------------------
      // Outcome handling
      // -----------------------------------------------------------------------
      first_point_attempt_counter++;
      if (!first_point_reached) {
        // we cannot reach the start point
        ROS_ERROR_STREAM(
            "MowingBehavior: (FIRST POINT) - Could not reach goal (first point). "
            "Planner Status was: "
            << current_status.state_);

        if (first_point_attempt_counter < config.max_first_point_attempts) {
          ROS_WARN_STREAM("MowingBehavior: (FIRST POINT) - Attempt " << first_point_attempt_counter << " / "
                                                                     << config.max_first_point_attempts
                                                                     << " Making a little pause ...");
          paused = true;
          update_actions();
        } else {
          // ---------------------------------------------------------------
          // Phase 3 recovery: retreat to the start of the previous mow path
          // before falling back to the trim logic.  Only attempted once per path.
          // ---------------------------------------------------------------
          if (localConfig.retreat_enabled && retreat_mowing_path_ != currentMowingPath && currentMowingPath > 0) {
            retreat_mowing_path_ = currentMowingPath;
            first_point_attempt_counter = 0;

            auto& prev_path = currentMowingPaths[currentMowingPath - 1];
            if (!prev_path.path.poses.empty()) {
              ROS_WARN_STREAM(
                  "MowingBehavior: (FIRST POINT) Retreating to start of previous path "
                  "(path "
                  << (currentMowingPath - 1) << ") to escape obstacle zone.");
              mbf_msgs::MoveBaseGoal retreat_goal;
              retreat_goal.target_pose = prev_path.path.poses.front();
              retreat_goal.controller = "FTCPlanner";
              mbfClient->sendGoal(retreat_goal);
              sleep(1);
              ros::Rate r_ret(10);
              while (ros::ok()) {
                auto st = mbfClient->getState();
                if (st.state_ == actionlib::SimpleClientGoalState::ACTIVE ||
                    st.state_ == actionlib::SimpleClientGoalState::PENDING) {
                  if (aborted) {
                    mbfClient->cancelAllGoals();
                    mowerEnabled = false;
                    return false;
                  }
                  if (requested_pause_flag) {
                    mbfClient->cancelAllGoals();
                    mowerEnabled = false;
                    return false;
                  }
                } else {
                  break;
                }
                r_ret.sleep();
              }
              ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) Retreat complete (state=" << mbfClient->getState().state_
                                                                                       << "), retrying first point.");
            }
            continue;  // retry the first-point navigation for currentMowingPath
          }

          // Trim: remove the first pose so the robot aims at the next one
          if (first_point_trim_counter < config.max_first_point_trim_attempts) {
            ROS_WARN_STREAM("MowingBehavior: (FIRST POINT) - Attempt "
                            << first_point_trim_counter << " / " << config.max_first_point_trim_attempts
                            << " Trimming first point off the beginning of the mow path.");
            currentMowingPathIndex++;
            first_point_trim_counter++;
            first_point_attempt_counter = 0;  // give it another <config.max_first_point_attempts> attempts
            paused = true;
            update_actions();
          } else {
            // Unable to reach any start of the mow path — give up
            ROS_ERROR_STREAM(
                "MowingBehavior: (FIRST POINT) Max retries reached, we are unable to reach any of the first points - "
                "aborting at index: "
                << currentMowingPathIndex << " path: " << currentMowingPath << " area: " << currentMowingArea);
            this->abort();
          }
        }
        continue;
      }

      mower_map::ClearNavPointSrv clear_nav_point_srv;
      clearNavPointClient.call(clear_nav_point_srv);

      // we have reached the start pose of the mow area, reset error handling values
      first_point_attempt_counter = 0;
      first_point_trim_counter = 0;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Execute the path segment and either drop it if we finished it successfully or trim it if we were aborted
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    {
      // enable mower (only when we reach the start not on the way to mowing already)
      mowerEnabled = true;

      mbf_msgs::ExePathGoal exePathGoal;
      nav_msgs::Path exePath;
      exePath.header = path.path.header;
      exePath.poses = std::vector<geometry_msgs::PoseStamped>(path.path.poses.begin() + currentMowingPathIndex,
                                                              path.path.poses.end());
      int exePathStartIndex = currentMowingPathIndex;
      exePathGoal.path = exePath;
      exePathGoal.angle_tolerance = 5.0 * (M_PI / 180.0);
      exePathGoal.dist_tolerance = 0.2;
      exePathGoal.tolerance_from_action = true;
      exePathGoal.controller = "FTCPlanner";

      ROS_INFO_STREAM("MowingBehavior: (MOW) First point reached - Executing mow path with "
                      << path.path.poses.size() << " poses, from index " << exePathStartIndex);
      mbfClientExePath->sendGoal(exePathGoal);
      sleep(1);
      actionlib::SimpleClientGoalState current_status(actionlib::SimpleClientGoalState::PENDING);
      ros::Rate r(10);

      // wait for path execution to finish
      while (ros::ok()) {
        current_status = mbfClientExePath->getState();
        if (current_status.state_ == actionlib::SimpleClientGoalState::ACTIVE ||
            current_status.state_ == actionlib::SimpleClientGoalState::PENDING) {
          // path is being executed, everything seems fine.
          // check if we should pause or abort mowing
          if (skip_area) {
            ROS_INFO_STREAM("MowingBehavior: (MOW) SKIP AREA was requested.");
            // remove all paths in current area and return true
            mowerEnabled = false;
            currentMowingPaths.clear();
            skip_area = false;
            return true;
          }
          if (skip_path) {
            skip_path = false;
            currentMowingPath++;
            currentMowingPathIndex = 0;
            return false;
          }
          if (aborted) {
            ROS_INFO_STREAM("MowingBehavior: (MOW) ABORT was requested - stopping path execution.");
            mbfClientExePath->cancelAllGoals();
            mowerEnabled = false;
            break;  // Trim path
          }
          if (requested_pause_flag) {
            ROS_INFO_STREAM("MowingBehavior: (MOW) PAUSE was requested - stopping path execution.");
            mbfClientExePath->cancelAllGoals();
            mowerEnabled = false;
            break;  // Trim path
          }
          if (current_status.state_ == actionlib::SimpleClientGoalState::ACTIVE) {
            // show progress
            int currentIndex = getCurrentMowPathIndex();
            if (currentIndex != -1) {
              currentMowingPathIndex = exePathStartIndex + currentIndex;
            }
            ROS_INFO_STREAM_THROTTLE(
                5, "MowingBehavior: (MOW) Progress: " << currentMowingPathIndex << "/" << path.path.poses.size());
            if (ros::Time::now() - last_checkpoint > ros::Duration(30.0)) checkpoint();
          }
        } else {
          ROS_INFO_STREAM("MowingBehavior: (MOW)  Got status " << current_status.state_
                                                               << " from MBF/FTCPlanner -> Stopping path execution.");
          // we're done, break out of the loop
          break;
        }
        r.sleep();
      }

      // Only skip/trim if goal execution began
      if (current_status.state_ != actionlib::SimpleClientGoalState::PENDING &&
          current_status.state_ != actionlib::SimpleClientGoalState::RECALLED) {
        ROS_INFO_STREAM(">> MowingBehavior: (MOW) PlannerGetProgress currentMowingPathIndex = "
                        << currentMowingPathIndex << " of " << path.path.poses.size());
        printNavState(current_status.state_);
        // if we have fully processed the segment or we have encountered an error, drop the path segment
        /* TODO: we can not trust the SUCCEEDED state because the planner sometimes says suceeded with
            the currentIndex far from the size of the poses ! (BUG in planner ?)
            instead we trust only the currentIndex vs. poses.size() */
        if (currentMowingPathIndex >= path.path.poses.size() ||
            (path.path.poses.size() - currentMowingPathIndex) < 5)  // fully mowed the path ?
        {
          ROS_INFO_STREAM("MowingBehavior: (MOW) Mow path finished, skipping to next mow path.");
          consecutive_obstacle_skips_ = 0;
          last_skipped_mowing_path_ = -1;
          currentMowingPath++;
          currentMowingPathIndex = 0;
          // continue with next segment
        } else {
          // we didnt drive all points in the mow path, so we trim the path and continue
          // Skip forward by obstacle_skip_count poses so the robot doesn't repeatedly attempt the same blocked segment

          // currentMowingPathIndex might be 0 if we never consumed one of the points, we advance at least 1 point
          if (currentMowingPathIndex == 0) currentMowingPathIndex++;
          if (!requested_pause_flag) {
            // Track consecutive skips within this segment; reset counter when segment changes
            if (last_skipped_mowing_path_ != currentMowingPath) {
              consecutive_obstacle_skips_ = 0;
              last_skipped_mowing_path_ = currentMowingPath;
            }
            consecutive_obstacle_skips_++;

            const int skip = std::max(1, (int)getConfig().obstacle_skip_count);
            currentMowingPathIndex = std::min((int)path.path.poses.size(), currentMowingPathIndex + skip);
            ROS_INFO_STREAM("MowingBehavior: (MOW) Obstacle skip "
                            << consecutive_obstacle_skips_ << ": advancing path index to " << currentMowingPathIndex
                            << " (+" << skip << ") of " << path.path.poses.size());

            // After too many consecutive skips in the same segment, abandon it entirely
            const int max_skips = getConfig().max_obstacle_skips_per_segment;
            if (max_skips > 0 && consecutive_obstacle_skips_ >= max_skips) {
              ROS_WARN_STREAM("MowingBehavior: (MOW) " << consecutive_obstacle_skips_
                                                       << " consecutive obstacle skips in segment " << currentMowingPath
                                                       << " — abandoning segment (max=" << max_skips << ").");
              consecutive_obstacle_skips_ = 0;
              last_skipped_mowing_path_ = -1;
              currentMowingPath++;
              currentMowingPathIndex = 0;
            } else if (currentMowingPathIndex >= (int)path.path.poses.size()) {
              // Skip exhausted the segment normally
              ROS_INFO_STREAM("MowingBehavior: (MOW) Skip exhausted path segment, moving to next.");
              currentMowingPath++;
              currentMowingPathIndex = 0;
            }
          }
        }
      }
    }
  }

  mowerEnabled = false;

  // true, if we have executed all paths
  return currentMowingPath >= currentMowingPaths.size();
}

void MowingBehavior::command_home() {
  if (shared_state->active_semiautomatic_task) {
    // We are in semiautomatic task, mark it as manually paused.
    ROS_INFO_STREAM("Manually pausing semiautomatic task");
    auto config = getConfig();
    config.manual_pause_mowing = true;
    setConfig(config);
  }
  if (paused) {
    // Request continue to wait for odom
    this->requestContinue();
    // Then instantly abort i.e. go to dock.
  }
  this->abort();
}

void MowingBehavior::command_start() {
  ROS_INFO_STREAM("MowingBehavior: MANUAL CONTINUE");
  auto config = getConfig();
  if (shared_state->active_semiautomatic_task && config.manual_pause_mowing) {
    // We are in semiautomatic task and paused, user wants to resume, so store that immediately.
    // This way, once we are docked the mower will continue as soon as all other conditions are g2g
    ROS_INFO_STREAM("Resuming semiautomatic task");
    config.manual_pause_mowing = false;
    setConfig(config);
  }
  this->requestContinue();
}

void MowingBehavior::command_s1() {
  ROS_INFO_STREAM("MowingBehavior: MANUAL PAUSED");
  this->requestPause();
}

void MowingBehavior::command_s2() {
  skip_area = true;
}

bool MowingBehavior::redirect_joystick() {
  return false;
}

uint8_t MowingBehavior::get_sub_state() {
  return 0;
}

uint8_t MowingBehavior::get_state() {
  return mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
}

int16_t MowingBehavior::get_current_area() {
  return currentMowingArea;
}

int16_t MowingBehavior::get_current_path() {
  return currentMowingPath;
}

int16_t MowingBehavior::get_current_path_index() {
  return currentMowingPathIndex;
}

MowingBehavior::MowingBehavior() {
  last_checkpoint = ros::Time(0.0);
  xbot_msgs::ActionInfo pause_action;
  pause_action.action_id = "pause";
  pause_action.enabled = false;
  pause_action.action_name = "Pause Mowing";

  xbot_msgs::ActionInfo continue_action;
  continue_action.action_id = "continue";
  continue_action.enabled = false;
  continue_action.action_name = "Continue Mowing";

  xbot_msgs::ActionInfo abort_mowing_action;
  abort_mowing_action.action_id = "abort_mowing";
  abort_mowing_action.enabled = false;
  abort_mowing_action.action_name = "Stop Mowing";

  xbot_msgs::ActionInfo skip_area_action;
  skip_area_action.action_id = "skip_area";
  skip_area_action.enabled = false;
  skip_area_action.action_name = "Skip Area";

  xbot_msgs::ActionInfo skip_path_action;
  skip_path_action.action_id = "skip_path";
  skip_path_action.enabled = false;
  skip_path_action.action_name = "Skip Path";

  actions.clear();
  actions.push_back(pause_action);
  actions.push_back(continue_action);
  actions.push_back(abort_mowing_action);
  actions.push_back(skip_area_action);
  actions.push_back(skip_path_action);
  restore_checkpoint();
}

void MowingBehavior::handle_action(std::string action) {
  if (action == "mower_logic:mowing/pause") {
    ROS_INFO_STREAM("got pause command");
    this->requestPause();
  } else if (action == "mower_logic:mowing/continue") {
    ROS_INFO_STREAM("got continue command");
    this->requestContinue();
  } else if (action == "mower_logic:mowing/abort_mowing") {
    ROS_INFO_STREAM("got abort mowing command");
    command_home();
  } else if (action == "mower_logic:mowing/skip_area") {
    ROS_INFO_STREAM("got skip_area command");
    skip_area = true;
  } else if (action == "mower_logic:mowing/skip_path") {
    ROS_INFO_STREAM("got skip_path command");
    skip_path = true;
  }
  update_actions();
}

void MowingBehavior::checkpoint() {
  rosbag::Bag bag;
  mower_logic::CheckPoint cp;
  cp.currentMowingPath = currentMowingPath;
  cp.currentMowingArea = currentMowingArea;
  cp.currentMowingPathIndex = currentMowingPathIndex;
  cp.currentMowingPlanDigest = currentMowingPlanDigest;
  cp.currentMowingAngleIncrementSum = currentMowingAngleIncrementSum;
  bag.open("checkpoint.bag", rosbag::bagmode::Write);
  bag.write("checkpoint", ros::Time::now(), cp);
  bag.close();
  last_checkpoint = ros::Time::now();
}

bool MowingBehavior::restore_checkpoint() {
  rosbag::Bag bag;
  bool found = false;
  try {
    bag.open("checkpoint.bag");
  } catch (rosbag::BagIOException& e) {
    // Checkpoint does not exist or is corrupt, start at the very beginning
    currentMowingArea = 0;
    currentMowingPath = 0;
    currentMowingPathIndex = 0;
    currentMowingAngleIncrementSum = 0;
    return false;
  }
  {
    rosbag::View view(bag, rosbag::TopicQuery("checkpoint"));
    for (rosbag::MessageInstance const m : view) {
      auto cp = m.instantiate<mower_logic::CheckPoint>();
      if (cp) {
        ROS_INFO_STREAM("Restoring checkpoint for plan ("
                        << cp->currentMowingPlanDigest << ")"
                        << " area: " << cp->currentMowingArea << " path: " << cp->currentMowingPath
                        << " index: " << cp->currentMowingPathIndex
                        << " angle increment sum: " << cp->currentMowingAngleIncrementSum);
        currentMowingPath = cp->currentMowingPath;
        currentMowingArea = cp->currentMowingArea;
        currentMowingPathIndex = cp->currentMowingPathIndex;
        currentMowingPlanDigest = cp->currentMowingPlanDigest;
        currentMowingAngleIncrementSum = cp->currentMowingAngleIncrementSum;
        found = true;
        break;
      }
    }
    bag.close();
  }
  return found;
}
