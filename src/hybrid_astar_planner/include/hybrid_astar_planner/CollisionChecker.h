// hybrid_astar_planner — costmap-based collision checking.
//
// This file is part of OpenMower (GNU GPLv3).
//
// Replaces Kurzer's precomputed collision-lookup table with a direct
// footprint-aware query against a costmap_2d::Costmap2D, restricted to a
// cropped planning window. Node coordinates are window-relative cells.
//
#ifndef HYBRID_ASTAR_PLANNER_COLLISIONCHECKER_H
#define HYBRID_ASTAR_PLANNER_COLLISIONCHECKER_H

#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>

#include <vector>

#include "hybrid_astar_planner/Node.h"

namespace hybrid_astar_planner {

class CollisionChecker {
 public:
  /*!
   * \param costmap   the (full) global costmap
   * \param footprint robot footprint polygon in base_link
   * \param ox,oy     window origin in absolute costmap cells
   * \param w,h       window size in cells
   */
  CollisionChecker(costmap_2d::Costmap2D* costmap, const std::vector<geometry_msgs::Point>& footprint, unsigned int ox,
                   unsigned int oy, int w, int h);

  /// Footprint-aware traversability at a 3D pose (window-relative cells + yaw).
  bool isTraversable(const Node3D* node) const;

  /// Cell occupancy for the 2D heuristic (window-relative cells).
  bool isTraversable(const Node2D* node) const;

 private:
  costmap_2d::Costmap2D* costmap_;
  // CostmapModel::footprintCost is non-const; mutable keeps isTraversable const.
  mutable base_local_planner::CostmapModel model_;
  std::vector<geometry_msgs::Point> footprint_;
  double inscribed_radius_;
  double circumscribed_radius_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  unsigned int ox_;
  unsigned int oy_;
  int w_;
  int h_;
};

}  // namespace hybrid_astar_planner

#endif  // HYBRID_ASTAR_PLANNER_COLLISIONCHECKER_H
