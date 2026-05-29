// hybrid_astar_planner — Hybrid A* global planner plugin for move_base_flex.
//
// This file is part of OpenMower and is distributed under the GNU GPLv3.
// The Hybrid A* search core (ported in later phases) derives from Karl Kurzer's
// path_planner, BSD-3-Clause, Copyright (c) 2017 Karl Kurzer.
//
#include "hybrid_astar_planner/HybridAstarPlanner.h"

#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <cmath>

// Register this planner as a nav_core::BaseGlobalPlanner plugin.
PLUGINLIB_EXPORT_CLASS(hybrid_astar_planner::HybridAstarPlanner, nav_core::BaseGlobalPlanner)

namespace hybrid_astar_planner {

HybridAstarPlanner::HybridAstarPlanner() = default;

HybridAstarPlanner::HybridAstarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  initialize(name, costmap_ros);
}

void HybridAstarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  if (initialized_) {
    ROS_WARN("HybridAstarPlanner: already initialized, skipping.");
    return;
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();

  ros::NodeHandle private_nh("~/" + name);
  plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

  initialized_ = true;
  ROS_INFO_STREAM("HybridAstarPlanner: initialized (frame="
                  << global_frame_ << ", " << costmap_->getSizeInCellsX() << "x" << costmap_->getSizeInCellsY()
                  << " cells, res=" << costmap_->getResolution() << "m). Phase A skeleton: straight-line plans.");
}

bool HybridAstarPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                  std::vector<geometry_msgs::PoseStamped>& plan) {
  if (!initialized_) {
    ROS_ERROR("HybridAstarPlanner: makePlan() called before initialize().");
    return false;
  }

  plan.clear();

  // ---------------------------------------------------------------------------
  // Phase A stub: straight-line interpolation from start to goal, with the
  // heading at every sample pointing along the segment (so the consumer/FTC
  // sees a sensible orientation). Phase B replaces this with the Hybrid A*
  // search over a cropped costmap window. The MBF/pluginlib/costmap plumbing
  // exercised here is identical to what the real planner will use.
  // ---------------------------------------------------------------------------
  const double dx = goal.pose.position.x - start.pose.position.x;
  const double dy = goal.pose.position.y - start.pose.position.y;
  const double dist = std::hypot(dx, dy);

  const double step = std::max(costmap_->getResolution(), 0.05);
  const int n = std::max(1, static_cast<int>(std::ceil(dist / step)));

  const double seg_yaw = (dist > 1e-6) ? std::atan2(dy, dx) : tf2::getYaw(goal.pose.orientation);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, seg_yaw);
  const geometry_msgs::Quaternion seg_quat = tf2::toMsg(q);

  for (int i = 0; i <= n; ++i) {
    const double f = static_cast<double>(i) / static_cast<double>(n);
    geometry_msgs::PoseStamped p;
    p.header.frame_id = global_frame_;
    p.header.stamp = ros::Time::now();
    p.pose.position.x = start.pose.position.x + f * dx;
    p.pose.position.y = start.pose.position.y + f * dy;
    p.pose.position.z = 0.0;
    p.pose.orientation = seg_quat;
    plan.push_back(p);
  }

  // Make the final pose carry the requested goal orientation.
  if (!plan.empty()) {
    plan.back().pose.orientation = goal.pose.orientation;
  }

  publishPlan(plan);
  ROS_INFO_STREAM("HybridAstarPlanner: (Phase A) straight-line plan with " << plan.size() << " poses over " << dist
                                                                           << "m.");
  return true;
}

void HybridAstarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
  nav_msgs::Path gui_path;
  gui_path.header.frame_id = global_frame_;
  gui_path.header.stamp = ros::Time::now();
  gui_path.poses = plan;
  plan_pub_.publish(gui_path);
}

}  // namespace hybrid_astar_planner
