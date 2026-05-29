// hybrid_astar_planner — Hybrid A* global planner plugin for move_base_flex.
//
// This file is part of OpenMower and is distributed under the GNU GPLv3.
// The Hybrid A* search core (ported in later phases) derives from Karl Kurzer's
// path_planner, BSD-3-Clause, Copyright (c) 2017 Karl Kurzer.
//
#ifndef HYBRID_ASTAR_PLANNER_H
#define HYBRID_ASTAR_PLANNER_H

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <ros/ros.h>

#include <string>
#include <vector>

namespace hybrid_astar_planner {

/*!
 * \brief Heading-aware global planner for a differential-drive robot, exposed
 *        as a nav_core::BaseGlobalPlanner so move_base_flex can load it as the
 *        "TransitionPlanner".
 *
 * Phase A: skeleton only — makePlan() returns a straight-line interpolation
 * with interpolated headings, proving the MBF/pluginlib/costmap integration.
 * The Hybrid A* search replaces the stub in Phase B.
 */
class HybridAstarPlanner : public nav_core::BaseGlobalPlanner {
 public:
  HybridAstarPlanner();
  HybridAstarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /// nav_core::BaseGlobalPlanner interface.
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
  bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override;

 private:
  /// Publish the plan as a nav_msgs/Path for RViz inspection.
  void publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

  costmap_2d::Costmap2DROS* costmap_ros_ = nullptr;
  costmap_2d::Costmap2D* costmap_ = nullptr;
  std::string global_frame_;
  ros::Publisher plan_pub_;
  bool initialized_ = false;
};

}  // namespace hybrid_astar_planner

#endif  // HYBRID_ASTAR_PLANNER_H
