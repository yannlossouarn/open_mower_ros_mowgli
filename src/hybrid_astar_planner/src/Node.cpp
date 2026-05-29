// hybrid_astar_planner — node successor generation.
//
// This file is part of OpenMower (GNU GPLv3). Derived from Karl Kurzer's
// path_planner (BSD-3-Clause, Copyright (c) 2017 Karl Kurzer); see LICENSE.Kurzer.
//
#include "hybrid_astar_planner/Node.h"

namespace hybrid_astar_planner {

// 8-connected grid for the 2D heuristic.
const int Node2D::dir = 8;
const int Node2D::dx[] = {-1, -1, 0, 1, 1, 1, 0, -1};
const int Node2D::dy[] = {0, 1, 1, 1, 0, -1, -1, -1};

Node2D* Node2D::createSuccessor(const int i) {
  return new Node2D(x + Node2D::dx[i], y + Node2D::dy[i], g, 0, this);
}

Node3D* Node3D::createSuccessor(const int i, const Primitives& mp) {
  // Forward-only (differential drive, no reverse): rotate the local primitive
  // displacement by the current heading and add the heading change.
  const float xSucc = x + mp.dx[i] * std::cos(t) - mp.dy[i] * std::sin(t);
  const float ySucc = y + mp.dx[i] * std::sin(t) + mp.dy[i] * std::cos(t);
  const float tSucc = normalizeHeadingRad(t + mp.dt[i]);
  return new Node3D(xSucc, ySucc, tSucc, g, 0, this, i);
}

}  // namespace hybrid_astar_planner
