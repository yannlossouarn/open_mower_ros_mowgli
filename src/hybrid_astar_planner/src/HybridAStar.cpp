// hybrid_astar_planner — Hybrid A* search.
//
// This file is part of OpenMower (GNU GPLv3). Derived from Karl Kurzer's
// path_planner (BSD-3-Clause, Copyright (c) 2017 Karl Kurzer); see LICENSE.Kurzer.
//
#include "hybrid_astar_planner/HybridAStar.h"

#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <boost/heap/binomial_heap.hpp>
#include <chrono>
#include <cmath>
#include <queue>
#include <utility>
#include <vector>

namespace hybrid_astar_planner {

namespace {

using SE2 = ompl::base::SE2StateSpace::StateType;
constexpr float kBig2D = 1e9f;

struct CompareNodes {
  bool operator()(const Node3D* lhs, const Node3D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
};

// Precompute the holonomic-with-obstacles 2D heuristic ONCE per plan: a single
// 8-connected Dijkstra from the goal cell over the cropped window. Stores the
// cost-to-goal (in cells) of every reachable cell in nodes2D[i].g; unreachable
// cells keep kBig2D. This replaces Kurzer's per-node on-demand 2D A* (which
// re-ran and reset the whole field on every cache miss).
void compute2DHeuristic(int goalX, int goalY, Node2D* nodes2D, int width, int height, CollisionChecker& cc) {
  for (int i = 0; i < width * height; ++i) {
    nodes2D[i] = Node2D(i % width, i / width, kBig2D, 0, nullptr);
  }
  if (goalX < 0 || goalX >= width || goalY < 0 || goalY >= height) return;

  using QE = std::pair<float, int>;  // (cost, idx)
  std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;
  const int gi = goalY * width + goalX;
  nodes2D[gi].setG(0.0f);
  pq.push({0.0f, gi});

  while (!pq.empty()) {
    const float c = pq.top().first;
    const int idx = pq.top().second;
    pq.pop();
    if (c > nodes2D[idx].getG() || nodes2D[idx].isClosed()) continue;
    nodes2D[idx].close();

    const int cx = idx % width;
    const int cy = idx / width;
    for (int k = 0; k < Node2D::dir; ++k) {
      const int nx = cx + Node2D::dx[k];
      const int ny = cy + Node2D::dy[k];
      if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
      Node2D probe(nx, ny, 0, 0, nullptr);
      if (!cc.isTraversable(&probe)) continue;
      const float step = std::sqrt(static_cast<float>(Node2D::dx[k] * Node2D::dx[k] + Node2D::dy[k] * Node2D::dy[k]));
      const float ng = nodes2D[idx].getG() + step;
      const int ni = ny * width + nx;
      if (ng < nodes2D[ni].getG()) {
        nodes2D[ni].setG(ng);
        pq.push({ng, ni});
      }
    }
  }
}

// Combined heuristic: max(constrained Dubins distance, precomputed 2D cost).
void updateH(Node3D& node, const Node3D& goal, const Node2D* nodes2D, int width, int height, double rCells) {
  ompl::base::DubinsStateSpace ds(rCells);
  ompl::base::State* a = ds.allocState();
  ompl::base::State* b = ds.allocState();
  a->as<SE2>()->setXY(node.getX(), node.getY());
  a->as<SE2>()->setYaw(node.getT());
  b->as<SE2>()->setXY(goal.getX(), goal.getY());
  b->as<SE2>()->setYaw(goal.getT());
  const float dubinsCost = static_cast<float>(ds.distance(a, b));
  ds.freeState(a);
  ds.freeState(b);

  const int sx = static_cast<int>(node.getX());
  const int sy = static_cast<int>(node.getY());
  float twoDCost = 0.0f;
  if (sx >= 0 && sx < width && sy >= 0 && sy < height) {
    twoDCost = nodes2D[sy * width + sx].getG();
    if (twoDCost >= kBig2D) twoDCost = 0.0f;  // unreachable in 2D: don't dominate Dubins
  }

  node.setH(std::max(dubinsCost, twoDCost));
}

// Analytic Dubins expansion from `from` to `goal`. On success fills `tail`
// (cells, start-exclusive..goal) and returns true; on any collision, false.
bool dubinsShot(const Node3D& from, const Node3D& goal, double rCells, const Params& params, double resolution,
                CollisionChecker& cc, std::vector<std::array<float, 3>>& tail) {
  ompl::base::DubinsStateSpace ds(rCells);
  ompl::base::State* a = ds.allocState();
  ompl::base::State* b = ds.allocState();
  ompl::base::State* s = ds.allocState();
  a->as<SE2>()->setXY(from.getX(), from.getY());
  a->as<SE2>()->setYaw(from.getT());
  b->as<SE2>()->setXY(goal.getX(), goal.getY());
  b->as<SE2>()->setYaw(goal.getT());

  const double length = ds.distance(a, b);  // cells
  const double stepCells = std::max(1e-3, params.dubins_step_size / resolution);
  const int n = std::max(1, static_cast<int>(length / stepCells));

  tail.clear();
  bool ok = true;
  for (int i = 1; i <= n; ++i) {
    const double f = static_cast<double>(i) / static_cast<double>(n);
    ds.interpolate(a, b, f, s);
    Node3D probe(static_cast<float>(s->as<SE2>()->getX()), static_cast<float>(s->as<SE2>()->getY()),
                 normalizeHeadingRad(static_cast<float>(s->as<SE2>()->getYaw())), 0, 0, nullptr);
    if (!cc.isTraversable(&probe)) {
      ok = false;
      break;
    }
    tail.push_back({probe.getX(), probe.getY(), probe.getT()});
  }

  ds.freeState(a);
  ds.freeState(b);
  ds.freeState(s);
  if (!ok) tail.clear();
  return ok;
}

}  // namespace

PlanResult hybridAStar(Node3D& start, const Node3D& goal, Node3D* nodes3D, Node2D* nodes2D, int width, int height,
                       const Params& params, const Primitives& prims, double resolution, CollisionChecker& cc) {
  PlanResult result;
  const float dh = static_cast<float>(params.deltaHeadingRad());
  const double rCells = params.min_turning_radius / resolution;
  const float shotRangeCells = static_cast<float>(params.dubins_shot_range / resolution);
  const int dir = Primitives::kDir;

  typedef boost::heap::binomial_heap<Node3D*, boost::heap::compare<CompareNodes>> PriorityQueue;
  PriorityQueue O;

  // Precompute the 2D holonomic heuristic field once (Dijkstra from goal cell).
  compute2DHeuristic(static_cast<int>(goal.getX()), static_cast<int>(goal.getY()), nodes2D, width, height, cc);

  updateH(start, goal, nodes2D, width, height, rCells);
  start.open();
  O.push(&start);
  int iPred = start.setIdx(width, height, dh);
  nodes3D[iPred] = start;

  int iterations = 0;
  long expansions = 0;
  const int analyticInterval = std::max(1, params.analytic_expansion_interval);
  const auto tStart = std::chrono::steady_clock::now();

  while (!O.empty()) {
    Node3D* nPred = O.top();
    iPred = nPred->setIdx(width, height, dh);
    iterations++;

    // Wall-clock budget: bound the worst case (e.g. an infeasible goal) so the
    // caller can fall back to a straight line quickly instead of hanging.
    if ((iterations & 0xFF) == 0) {
      const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - tStart).count();
      if (elapsed > params.max_planning_time) {
        return result;  // not found within budget
      }
    }

    if (nodes3D[iPred].isClosed()) {
      O.pop();
      continue;
    }

    if (nodes3D[iPred].isOpen()) {
      nodes3D[iPred].close();
      O.pop();
      ++expansions;

      // Goal reached (or iteration budget exhausted -> return best-so-far).
      if (nPred->equals(goal, dh) || iterations > params.max_iterations) {
        result.found = nPred->equals(goal, dh);
        result.node = &nodes3D[iPred];
        return result;
      }

      // Analytic Dubins expansion (throttled: it is costly, so only attempt it
      // every Nth expansion — the search still converges via grid expansion).
      if (params.dubins_shot && (expansions % analyticInterval == 0) && nPred->isInRange(goal, shotRangeCells)) {
        std::vector<std::array<float, 3>> tail;
        if (dubinsShot(*nPred, goal, rCells, params, resolution, cc, tail)) {
          result.found = true;
          result.node = &nodes3D[iPred];
          result.tail = std::move(tail);
          return result;
        }
      }

      // Forward simulation.
      for (int i = 0; i < dir; ++i) {
        Node3D* nSucc = nPred->createSuccessor(i, prims);
        int iSucc = nSucc->setIdx(width, height, dh);

        if (nSucc->isOnGrid(width, height, dh, params.headings) && cc.isTraversable(nSucc)) {
          if (!nodes3D[iSucc].isClosed() || iPred == iSucc) {
            nSucc->updateG(prims, params);
            float newG = nSucc->getG();

            if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].getG() || iPred == iSucc) {
              updateH(*nSucc, goal, nodes2D, width, height, rCells);

              if (iPred == iSucc && nSucc->getC() > nPred->getC() + static_cast<float>(params.tie_breaker)) {
                delete nSucc;
                continue;
              } else if (iPred == iSucc && nSucc->getC() <= nPred->getC() + static_cast<float>(params.tie_breaker)) {
                nSucc->setPred(nPred->getPred());
              }

              nSucc->open();
              nodes3D[iSucc] = *nSucc;
              O.push(&nodes3D[iSucc]);
              delete nSucc;
            } else {
              delete nSucc;
            }
          } else {
            delete nSucc;
          }
        } else {
          delete nSucc;
        }
      }
    }
  }

  return result;  // not found
}

}  // namespace hybrid_astar_planner
