// hybrid_astar_planner — Hybrid A* search.
//
// This file is part of OpenMower (GNU GPLv3). Derived from Karl Kurzer's
// path_planner (BSD-3-Clause, Copyright (c) 2017 Karl Kurzer); see LICENSE.Kurzer.
//
#ifndef HYBRID_ASTAR_PLANNER_HYBRIDASTAR_H
#define HYBRID_ASTAR_PLANNER_HYBRIDASTAR_H

#include <array>
#include <vector>

#include "hybrid_astar_planner/CollisionChecker.h"
#include "hybrid_astar_planner/Node.h"
#include "hybrid_astar_planner/Params.h"

namespace hybrid_astar_planner {

struct PlanResult {
  bool found = false;
  const Node3D* node = nullptr;            ///< goal-reaching node in nodes3D; backtrace via getPred()
  std::vector<std::array<float, 3>> tail;  ///< analytic Dubins tail in cells (x,y,theta), start-exclusive..goal
};

/*!
 * \brief Run Hybrid A* over a cropped window.
 *
 * \param nodes3D caller-allocated array of size width*height*headings
 * \param nodes2D caller-allocated array of size width*height
 * \param resolution costmap resolution [m/cell] (for Dubins radius in cells)
 */
PlanResult hybridAStar(Node3D& start, const Node3D& goal, Node3D* nodes3D, Node2D* nodes2D, int width, int height,
                       const Params& params, const Primitives& prims, double resolution, CollisionChecker& cc);

}  // namespace hybrid_astar_planner

#endif  // HYBRID_ASTAR_PLANNER_HYBRIDASTAR_H
