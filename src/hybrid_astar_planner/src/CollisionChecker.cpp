// hybrid_astar_planner — costmap-based collision checking.
//
// This file is part of OpenMower (GNU GPLv3).
//
#include "hybrid_astar_planner/CollisionChecker.h"

#include <costmap_2d/cost_values.h>

#include <cmath>
#include <limits>

namespace hybrid_astar_planner {

namespace {
// Compute inscribed/circumscribed radii from a footprint polygon (base_link
// origin). Mirrors costmap_2d::calculateMinAndMaxDistances.
void footprintRadii(const std::vector<geometry_msgs::Point>& fp, double& inscribed, double& circumscribed) {
  inscribed = std::numeric_limits<double>::max();
  circumscribed = 0.0;
  if (fp.size() <= 2) {
    inscribed = 0.0;
    return;
  }
  for (size_t i = 0; i < fp.size(); ++i) {
    const double vx = fp[i].x, vy = fp[i].y;
    const size_t j = (i + 1) % fp.size();
    const double wx = fp[j].x, wy = fp[j].y;

    circumscribed = std::max(circumscribed, std::hypot(vx, vy));

    // distance from origin to segment (vx,vy)-(wx,wy)
    const double dx = wx - vx, dy = wy - vy;
    const double len2 = dx * dx + dy * dy;
    double dist;
    if (len2 < 1e-9) {
      dist = std::hypot(vx, vy);
    } else {
      double tproj = -(vx * dx + vy * dy) / len2;
      tproj = std::max(0.0, std::min(1.0, tproj));
      const double px = vx + tproj * dx, py = vy + tproj * dy;
      dist = std::hypot(px, py);
    }
    inscribed = std::min(inscribed, dist);
  }
}
}  // namespace

CollisionChecker::CollisionChecker(costmap_2d::Costmap2D* costmap, const std::vector<geometry_msgs::Point>& footprint,
                                   unsigned int ox, unsigned int oy, int w, int h)
    : costmap_(costmap),
      model_(*costmap),
      footprint_(footprint),
      resolution_(costmap->getResolution()),
      origin_x_(costmap->getOriginX()),
      origin_y_(costmap->getOriginY()),
      ox_(ox),
      oy_(oy),
      w_(w),
      h_(h) {
  footprintRadii(footprint_, inscribed_radius_, circumscribed_radius_);
}

bool CollisionChecker::isTraversable(const Node3D* node) const {
  // Window cell -> world (cell centre).
  const double wx = origin_x_ + (ox_ + node->getX() + 0.5) * resolution_;
  const double wy = origin_y_ + (oy_ + node->getY() + 0.5) * resolution_;

  // CostmapModel::footprintCost returns >= 0 for a valid cost, or a negative
  // value for off-map / collision. We treat any negative result as blocked.
  const double cost = model_.footprintCost(wx, wy, node->getT(), footprint_, inscribed_radius_, circumscribed_radius_);
  return cost >= 0.0;
}

unsigned char CollisionChecker::costAt(const Node3D* node) const {
  const unsigned int mx = ox_ + static_cast<unsigned int>(node->getX());
  const unsigned int my = oy_ + static_cast<unsigned int>(node->getY());
  if (mx >= costmap_->getSizeInCellsX() || my >= costmap_->getSizeInCellsY()) {
    return costmap_2d::LETHAL_OBSTACLE;
  }
  return costmap_->getCost(mx, my);
}

unsigned char CollisionChecker::costAt(const Node2D* node) const {
  const unsigned int mx = ox_ + static_cast<unsigned int>(node->getX());
  const unsigned int my = oy_ + static_cast<unsigned int>(node->getY());
  if (mx >= costmap_->getSizeInCellsX() || my >= costmap_->getSizeInCellsY()) {
    return costmap_2d::LETHAL_OBSTACLE;
  }
  return costmap_->getCost(mx, my);
}

bool CollisionChecker::isTraversable(const Node2D* node) const {
  const unsigned int mx = ox_ + node->getX();
  const unsigned int my = oy_ + node->getY();
  if (mx >= costmap_->getSizeInCellsX() || my >= costmap_->getSizeInCellsY()) {
    return false;
  }
  const unsigned char cost = costmap_->getCost(mx, my);
  return cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

}  // namespace hybrid_astar_planner
