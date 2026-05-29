// hybrid_astar_planner — search configuration and motion primitives.
//
// This file is part of OpenMower (GNU GPLv3). The Hybrid A* search it
// configures derives from Karl Kurzer's path_planner (BSD-3-Clause,
// Copyright (c) 2017 Karl Kurzer); see LICENSE.Kurzer.
//
#ifndef HYBRID_ASTAR_PLANNER_PARAMS_H
#define HYBRID_ASTAR_PLANNER_PARAMS_H

#include <cmath>

namespace hybrid_astar_planner {

/*!
 * \brief Search configuration. Defaults are tuned for a differential-drive
 *        mower; the plugin exposes the important ones as ROS params.
 *
 * The robot is treated as a non-holonomic vehicle with a (small) minimum
 * turning radius so the search produces smooth curves into the goal heading
 * rather than in-place rotations near obstacles.
 */
struct Params {
  // Kinematics
  double min_turning_radius = 0.6;  ///< [m]
  double step_size = 0.30;          ///< [m] arc length per motion-primitive expansion
  int headings = 72;                ///< heading discretisation (bins over 2*pi)

  // Search limits
  int max_iterations = 30000;
  double max_planning_time = 2.0;  ///< [s] wall-clock budget; bounds worst case on the Pi

  // Cost penalties (unitless multipliers on the base step cost)
  double penalty_turning = 1.05;

  // Analytic expansion (Dubins shot)
  bool dubins_shot = true;
  double dubins_shot_range = 4.0;       ///< [m] attempt a shot when within this range of goal
  double dubins_step_size = 0.10;       ///< [m] sampling step along the Dubins path (collision check)
  int analytic_expansion_interval = 1;  ///< attempt a Dubins shot every Nth expansion (1 = always)

  // Tie-breaker so a successor can be placed in a cell already holding its
  // predecessor (see Kurzer's constants.h discussion).
  double tie_breaker = 0.01;

  /// Window margin added around the start/goal bounding box.
  double window_margin = 2.0;  ///< [m]

  double deltaHeadingRad() const {
    return 2.0 * M_PI / static_cast<double>(headings);
  }
};

/*!
 * \brief Forward motion primitives in CELL units (depend on costmap
 *        resolution, so they are recomputed per plan). Index 0 is straight,
 *        1 is a left turn, 2 is a right turn. Differential drive => no reverse.
 */
struct Primitives {
  static constexpr int kDir = 3;
  float dx[3];
  float dy[3];
  float dt[3];
};

/// Build the cell-unit primitives for a given resolution.
inline Primitives makePrimitives(const Params& p, double resolution) {
  Primitives mp;
  const double R = p.min_turning_radius;
  const double L = p.step_size;  // arc length [m]
  const double dtheta = L / R;   // heading change per turn step [rad]

  // straight
  mp.dx[0] = static_cast<float>(L / resolution);
  mp.dy[0] = 0.0f;
  mp.dt[0] = 0.0f;

  // left turn (+dtheta): local displacement (R sin dtheta, +R(1-cos dtheta))
  mp.dx[1] = static_cast<float>(R * std::sin(dtheta) / resolution);
  mp.dy[1] = static_cast<float>(R * (1.0 - std::cos(dtheta)) / resolution);
  mp.dt[1] = static_cast<float>(dtheta);

  // right turn (-dtheta): mirror of left
  mp.dx[2] = mp.dx[1];
  mp.dy[2] = -mp.dy[1];
  mp.dt[2] = -static_cast<float>(dtheta);

  return mp;
}

}  // namespace hybrid_astar_planner

#endif  // HYBRID_ASTAR_PLANNER_PARAMS_H
