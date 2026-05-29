// hybrid_astar_planner — Hybrid A* global planner plugin for move_base_flex.
//
// This file is part of OpenMower and is distributed under the GNU GPLv3.
// The Hybrid A* search core derives from Karl Kurzer's path_planner,
// BSD-3-Clause, Copyright (c) 2017 Karl Kurzer; see LICENSE.Kurzer.
//
#include "hybrid_astar_planner/HybridAstarPlanner.h"

#include <costmap_2d/cost_values.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>
#include <memory>

#include "hybrid_astar_planner/HybridAStar.h"

PLUGINLIB_EXPORT_CLASS(hybrid_astar_planner::HybridAstarPlanner, nav_core::BaseGlobalPlanner)

namespace hybrid_astar_planner {

namespace {
constexpr long kMaxNodes3D = 3000000;  // ~150 MB of Node3D; guard against OOM on the Pi
}

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

  ros::NodeHandle pnh("~/" + name);
  pnh.param("min_turning_radius", params_.min_turning_radius, params_.min_turning_radius);
  pnh.param("step_size", params_.step_size, params_.step_size);
  pnh.param("headings", params_.headings, params_.headings);
  pnh.param("max_iterations", params_.max_iterations, params_.max_iterations);
  pnh.param("max_planning_time", params_.max_planning_time, params_.max_planning_time);
  pnh.param("penalty_turning", params_.penalty_turning, params_.penalty_turning);
  pnh.param("dubins_shot", params_.dubins_shot, params_.dubins_shot);
  pnh.param("dubins_shot_range", params_.dubins_shot_range, params_.dubins_shot_range);
  pnh.param("dubins_step_size", params_.dubins_step_size, params_.dubins_step_size);
  pnh.param("analytic_expansion_interval", params_.analytic_expansion_interval, params_.analytic_expansion_interval);
  pnh.param("window_margin", params_.window_margin, params_.window_margin);
  pnh.param("robot_front", params_.robot_front, params_.robot_front);
  pnh.param("robot_rear", params_.robot_rear, params_.robot_rear);
  pnh.param("robot_half_width", params_.robot_half_width, params_.robot_half_width);
  pnh.param("weight_costmap", params_.weight_costmap, params_.weight_costmap);
  pnh.param("shot_max_cost", params_.shot_max_cost, params_.shot_max_cost);

  // Build the explicit footprint polygon (base_link, x forward). The costmap's
  // own footprint is unreliable in this stack, which made collision checks
  // degrade to a single centre cell; using our own restores body-aware checks.
  auto pt = [](double x, double y) {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0.0;
    return p;
  };
  footprint_ = {pt(params_.robot_front, params_.robot_half_width), pt(params_.robot_front, -params_.robot_half_width),
                pt(-params_.robot_rear, -params_.robot_half_width), pt(-params_.robot_rear, params_.robot_half_width)};

  plan_pub_ = pnh.advertise<nav_msgs::Path>("plan", 1);

  initialized_ = true;
  ROS_INFO_STREAM("HybridAstarPlanner: initialized (frame="
                  << global_frame_ << ", " << costmap_->getSizeInCellsX() << "x" << costmap_->getSizeInCellsY()
                  << " cells, res=" << costmap_->getResolution() << "m, R=" << params_.min_turning_radius
                  << "m, step=" << params_.step_size << "m, headings=" << params_.headings
                  << ", weight_costmap=" << params_.weight_costmap << ", footprint=" << params_.robot_front << "/"
                  << params_.robot_rear << "/" << params_.robot_half_width << ").");
}

bool HybridAstarPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                  std::vector<geometry_msgs::PoseStamped>& plan) {
  if (!initialized_) {
    ROS_ERROR("HybridAstarPlanner: makePlan() called before initialize().");
    return false;
  }

  plan.clear();
  if (!runHybridAStar(start, goal, plan)) {
    // Fail cleanly rather than emitting an un-collision-checked straight line:
    // as a global planner that could route a controller through an obstacle.
    // Callers (e.g. MowingBehavior) handle planning failure with their own
    // fallbacks (GlobalPlanner / straight-line glide-in).
    ROS_WARN("HybridAstarPlanner: planning failed; returning no plan.");
    return false;
  }

  publishPlan(plan);
  return !plan.empty();
}

bool HybridAstarPlanner::runHybridAStar(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                        std::vector<geometry_msgs::PoseStamped>& plan) {
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  const double res = costmap_->getResolution();
  const double ox_w = costmap_->getOriginX();
  const double oy_w = costmap_->getOriginY();

  // World -> absolute costmap cells.
  unsigned int sx, sy, gx, gy;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, sx, sy) ||
      !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy)) {
    ROS_WARN("HybridAstarPlanner: start or goal is outside the costmap.");
    return false;
  }

  // Cropped window: bounding box of start/goal + margin, clamped to the map.
  const int margin = std::max(1, static_cast<int>(std::ceil(params_.window_margin / res)));
  const int sizeX = static_cast<int>(costmap_->getSizeInCellsX());
  const int sizeY = static_cast<int>(costmap_->getSizeInCellsY());
  const int minx = std::max(0, static_cast<int>(std::min(sx, gx)) - margin);
  const int miny = std::max(0, static_cast<int>(std::min(sy, gy)) - margin);
  const int maxx = std::min(sizeX - 1, static_cast<int>(std::max(sx, gx)) + margin);
  const int maxy = std::min(sizeY - 1, static_cast<int>(std::max(sy, gy)) + margin);
  const int width = maxx - minx + 1;
  const int height = maxy - miny + 1;
  if (width < 2 || height < 2) {
    ROS_WARN("HybridAstarPlanner: degenerate planning window (%dx%d).", width, height);
    return false;
  }

  const long n3d = static_cast<long>(width) * height * params_.headings;
  if (n3d > kMaxNodes3D) {
    ROS_WARN("HybridAstarPlanner: window too large (%ld 3D nodes > %ld cap); falling back.", n3d, kMaxNodes3D);
    return false;
  }

  // Window-relative continuous cell coordinates.
  auto toWindowCell = [&](double wx, double wy, float& cx, float& cy) {
    cx = static_cast<float>((wx - ox_w) / res - minx);
    cy = static_cast<float>((wy - oy_w) / res - miny);
  };
  float scx, scy, gcx, gcy;
  toWindowCell(start.pose.position.x, start.pose.position.y, scx, scy);
  toWindowCell(goal.pose.position.x, goal.pose.position.y, gcx, gcy);

  Node3D startNode(scx, scy, normalizeHeadingRad(static_cast<float>(tf2::getYaw(start.pose.orientation))), 0, 0,
                   nullptr);
  Node3D goalNode(gcx, gcy, normalizeHeadingRad(static_cast<float>(tf2::getYaw(goal.pose.orientation))), 0, 0, nullptr);

  const Primitives prims = makePrimitives(params_, res);
  CollisionChecker cc(costmap_, footprint_, minx, miny, width, height);

  // Fast goal-feasibility precheck: if the robot footprint at the goal pose is
  // in collision (a common case for obstacle-adjacent mow-strip starts), no
  // plan can terminate there — fail fast rather than exhausting the search.
  Node3D goalProbe(gcx, gcy, normalizeHeadingRad(static_cast<float>(tf2::getYaw(goal.pose.orientation))), 0, 0,
                   nullptr);
  if (!cc.isTraversable(&goalProbe)) {
    ROS_WARN("HybridAstarPlanner: goal pose footprint is in collision; cannot plan to it.");
    return false;
  }

  std::unique_ptr<Node3D[]> nodes3D(new Node3D[n3d]());
  std::unique_ptr<Node2D[]> nodes2D(new Node2D[static_cast<long>(width) * height]());

  const PlanResult result =
      hybridAStar(startNode, goalNode, nodes3D.get(), nodes2D.get(), width, height, params_, prims, res, cc);

  if (!result.node) {
    return false;
  }
  if (!result.found) {
    ROS_WARN("HybridAstarPlanner: search exhausted without reaching the goal.");
    return false;
  }

  // Backtrace the search chain (goal-side node back to start), then reverse.
  // Guard against a predecessor cycle (the same-cell rewire can, in rare
  // geometries, produce a loop) so we never spin here.
  std::vector<std::array<float, 3>> cells;  // (x, y, theta) in window cells, start..node
  const size_t maxChain = static_cast<size_t>(n3d) + 16;
  size_t guard = 0;
  for (const Node3D* n = result.node; n != nullptr && guard < maxChain; n = n->getPred(), ++guard) {
    cells.push_back({n->getX(), n->getY(), n->getT()});
  }
  if (guard >= maxChain) {
    ROS_WARN("HybridAstarPlanner: predecessor chain exceeded bound (cycle?); discarding plan.");
    return false;
  }
  std::reverse(cells.begin(), cells.end());
  // Append the analytic Dubins tail (already start-exclusive..goal order).
  for (const auto& c : result.tail) cells.push_back(c);

  // Window cells -> world poses.
  const ros::Time now = ros::Time::now();
  plan.reserve(cells.size() + 1);
  for (const auto& c : cells) {
    geometry_msgs::PoseStamped p;
    p.header.frame_id = global_frame_;
    p.header.stamp = now;
    p.pose.position.x = ox_w + (minx + c[0] + 0.5) * res;
    p.pose.position.y = oy_w + (miny + c[1] + 0.5) * res;
    p.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, c[2]);
    p.pose.orientation = tf2::toMsg(q);
    plan.push_back(p);
  }

  // Ensure the path terminates exactly at the requested goal pose.
  geometry_msgs::PoseStamped goal_stamped = goal;
  goal_stamped.header.frame_id = global_frame_;
  goal_stamped.header.stamp = now;
  plan.push_back(goal_stamped);

  ROS_INFO_STREAM("HybridAstarPlanner: plan found — " << plan.size() << " poses (window " << width << "x" << height
                                                      << " cells, " << result.tail.size() << " analytic tail).");
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
