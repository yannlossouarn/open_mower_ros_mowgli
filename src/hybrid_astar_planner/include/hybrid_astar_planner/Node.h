// hybrid_astar_planner — 2D (holonomic heuristic) and 3D (search) nodes.
//
// This file is part of OpenMower (GNU GPLv3). Derived from Karl Kurzer's
// path_planner (BSD-3-Clause, Copyright (c) 2017 Karl Kurzer); see
// LICENSE.Kurzer. Coordinates are in grid CELLS, window-relative.
//
#ifndef HYBRID_ASTAR_PLANNER_NODE_H
#define HYBRID_ASTAR_PLANNER_NODE_H

#include <cmath>

#include "hybrid_astar_planner/Params.h"

namespace hybrid_astar_planner {

/// Normalise a heading in radians to [0, 2*pi).
inline float normalizeHeadingRad(float t) {
  if (t < 0.0f) {
    t = t - 2.0f * static_cast<float>(M_PI) * static_cast<int>(t / (2.0f * static_cast<float>(M_PI)));
    return 2.0f * static_cast<float>(M_PI) + t;
  }
  return t - 2.0f * static_cast<float>(M_PI) * static_cast<int>(t / (2.0f * static_cast<float>(M_PI)));
}

/*!
 * \brief 2D node for the holonomic-with-obstacles (2D A*) heuristic.
 *        Position is a discrete cell (x, y), window-relative.
 */
class Node2D {
 public:
  Node2D() : Node2D(0, 0, 0, 0, nullptr) {
  }
  Node2D(int x, int y, float g, float h, Node2D* pred)
      : x(x), y(y), g(g), h(h), idx(-1), o(false), c(false), d(false), pred(pred) {
  }

  int getX() const {
    return x;
  }
  int getY() const {
    return y;
  }
  float getG() const {
    return g;
  }
  float getC() const {
    return g + h;
  }
  int getIdx() const {
    return idx;
  }
  bool isOpen() const {
    return o;
  }
  bool isClosed() const {
    return c;
  }
  bool isDiscovered() const {
    return d;
  }

  void setG(const float& g) {
    this->g = g;
  }
  int setIdx(int width) {
    idx = y * width + x;
    return idx;
  }
  void open() {
    o = true;
    c = false;
  }
  void close() {
    c = true;
    o = false;
  }
  void reset() {
    c = false;
    o = false;
  }
  void discover() {
    d = true;
  }

  void updateG() {
    g += movementCost(*pred);
    d = true;
  }
  void updateH(const Node2D& goal) {
    h = movementCost(goal);
  }
  float movementCost(const Node2D& p) const {
    return std::sqrt((x - p.x) * (x - p.x) + (y - p.y) * (y - p.y));
  }

  bool operator==(const Node2D& rhs) const {
    return x == rhs.x && y == rhs.y;
  }
  bool isOnGrid(const int width, const int height) const {
    return x >= 0 && x < width && y >= 0 && y < height;
  }
  Node2D* createSuccessor(const int i);

  static const int dir;
  static const int dx[];
  static const int dy[];

 private:
  int x;
  int y;
  float g;
  float h;
  int idx;
  bool o;
  bool c;
  bool d;
  Node2D* pred;
};

/*!
 * \brief 3D search node: continuous cell position (x, y) plus heading t [rad].
 */
class Node3D {
 public:
  Node3D() : Node3D(0, 0, 0, 0, 0, nullptr) {
  }
  Node3D(float x, float y, float t, float g, float h, const Node3D* pred, int prim = 0)
      : x(x), y(y), t(t), g(g), h(h), idx(-1), o(false), c(false), prim(prim), pred(pred) {
  }

  float getX() const {
    return x;
  }
  float getY() const {
    return y;
  }
  float getT() const {
    return t;
  }
  float getG() const {
    return g;
  }
  float getC() const {
    return g + h;
  }
  int getIdx() const {
    return idx;
  }
  int getPrim() const {
    return prim;
  }
  bool isOpen() const {
    return o;
  }
  bool isClosed() const {
    return c;
  }
  const Node3D* getPred() const {
    return pred;
  }

  void setX(const float& v) {
    x = v;
  }
  void setY(const float& v) {
    y = v;
  }
  void setT(const float& v) {
    t = v;
  }
  void setG(const float& v) {
    g = v;
  }
  void setH(const float& v) {
    h = v;
  }

  /// Cell/heading index into the 3D node array.
  int setIdx(int width, int height, float deltaHeadingRad) {
    int hIdx = static_cast<int>(t / deltaHeadingRad);
    idx = hIdx * width * height + static_cast<int>(y) * width + static_cast<int>(x);
    return idx;
  }
  void open() {
    o = true;
    c = false;
  }
  void close() {
    c = true;
    o = false;
  }
  void setPred(const Node3D* p) {
    pred = p;
  }

  /// Increase cost-so-far for this successor coming from its predecessor.
  /// cellCost is the costmap cost (0..254) at this node, penalised so the
  /// search prefers clearance over length.
  void updateG(const Primitives& mp, const Params& params, float cellCost) {
    const float move = (pred && pred->prim != prim) ? mp.dx[0] * static_cast<float>(params.penalty_turning) : mp.dx[0];
    const float costPenalty = static_cast<float>(params.weight_costmap) * (cellCost / 254.0f) * mp.dx[0];
    g += move + costPenalty;
  }

  /// Equal if same cell and heading within one discretisation step.
  bool equals(const Node3D& rhs, float deltaHeadingRad) const {
    return static_cast<int>(x) == static_cast<int>(rhs.x) && static_cast<int>(y) == static_cast<int>(rhs.y) &&
           (std::abs(t - rhs.t) <= deltaHeadingRad ||
            std::abs(t - rhs.t) >= (2.0f * static_cast<float>(M_PI) - deltaHeadingRad));
  }

  bool isOnGrid(const int width, const int height, float deltaHeadingRad, int headings) const {
    return x >= 0 && x < width && y >= 0 && y < height && static_cast<int>(t / deltaHeadingRad) >= 0 &&
           static_cast<int>(t / deltaHeadingRad) < headings;
  }

  bool isInRange(const Node3D& goal, float rangeCells) const {
    float dx = std::abs(x - goal.x);
    float dy = std::abs(y - goal.y);
    return (dx * dx + dy * dy) < (rangeCells * rangeCells);
  }

  /// Create the i-th forward successor in continuous space.
  Node3D* createSuccessor(const int i, const Primitives& mp);

 private:
  float x;
  float y;
  float t;
  float g;
  float h;
  int idx;
  bool o;
  bool c;
  int prim;
  const Node3D* pred;
};

}  // namespace hybrid_astar_planner

#endif  // HYBRID_ASTAR_PLANNER_NODE_H
