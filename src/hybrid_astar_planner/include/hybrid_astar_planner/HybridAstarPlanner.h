// hybrid_astar_planner — Hybrid A* global planner plugin for move_base_flex.
//
// This file is part of OpenMower and is distributed under the GNU GPLv3.
// The Hybrid A* search core derives from Karl Kurzer's path_planner,
// BSD-3-Clause, Copyright (c) 2017 Karl Kurzer; see LICENSE.Kurzer.
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

#include "hybrid_astar_planner/Params.h"

namespace hybrid_astar_planner {

/*!
 * \brief Heading-aware global planner for a differential-drive robot, exposed
 *        as a nav_core::BaseGlobalPlanner so move_base_flex can load it as the
 *        "TransitionPlanner". Plans a kinematically feasible Hybrid A* path so
 *        the robot reaches the goal pose with the correct orientation without
 *        large in-place rotations near obstacles.
 */
class HybridAstarPlanner : public nav_core::BaseGlobalPlanner {
 public:
  HybridAstarPlanner();
  HybridAstarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
  bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override;

 private:
  /// Run Hybrid A* over a cropped window. Returns false (so makePlan can fall
  /// back to a straight line) if the search fails or inputs are off-map.
  bool runHybridAStar(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan);

  /// Straight-line interpolation fallback (also used when the search fails).
  void straightLinePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                        std::vector<geometry_msgs::PoseStamped>& plan);

  void publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

  costmap_2d::Costmap2DROS* costmap_ros_ = nullptr;
  costmap_2d::Costmap2D* costmap_ = nullptr;
  std::string global_frame_;
  ros::Publisher plan_pub_;
  Params params_;
  bool initialized_ = false;
};

}  // namespace hybrid_astar_planner

#endif  // HYBRID_ASTAR_PLANNER_H
