//
// Created by Clemens Elflein on 27.08.21.
//

#include "ros/ros.h"

#include <boost/range/adaptor/reversed.hpp>

#include "ExPolygon.hpp"
#include "Polyline.hpp"
#include "Fill/FillRectilinear.hpp"
#include "Fill/FillConcentric.hpp"


#include "slic3r_coverage_planner/PlanPath.h"
#include "visualization_msgs/MarkerArray.h"
#include "Surface.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <Fill/FillPlanePath.hpp>
#include <PerimeterGenerator.hpp>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "ClipperUtils.hpp"
#include "ExtrusionEntityCollection.hpp"
#include <cmath>


bool visualize_plan;
bool omega_detection_enabled;
double omega_max_pair_distance;
double omega_cusp_angle_deg;
double omega_approach_angle_deg;
ros::Publisher marker_array_publisher;


void
createMarkers(const slic3r_coverage_planner::PlanPathRequest &planning_request,
              const slic3r_coverage_planner::PlanPathResponse &planning_result,
              visualization_msgs::MarkerArray &markerArray) {

    std::vector<std_msgs::ColorRGBA> colors;

    {
        std_msgs::ColorRGBA color;
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::ColorRGBA color;
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::ColorRGBA color;
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::ColorRGBA color;
        color.r = 1.0;
        color.g = 1.0;
        color.b = 0.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::ColorRGBA color;
        color.r = 1.0;
        color.g = 0.0;
        color.b = 1.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::ColorRGBA color;
        color.r = 0.0;
        color.g = 1.0;
        color.b = 1.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::ColorRGBA color;
        color.r = 1.0;
        color.g = 1.0;
        color.b = 1.0;
        color.a = 1.0;
        colors.push_back(color);
    }

    // Create markers for the input polygon
    {
        // Walk through the paths we send to the navigation stack
        auto &path = planning_request.outline.points;
        // Each group gets a single line strip as marker
        visualization_msgs::Marker marker;

        marker.header.frame_id = "map";
        marker.ns = "mower_map_service_lines";
        marker.id = static_cast<int>(markerArray.markers.size());
        marker.frame_locked = true;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.color = colors[0];
        marker.pose.orientation.w = 1;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.02;

        // Add the points to the line strip
        for (auto &point: path) {

            geometry_msgs::Point vpt;
            vpt.x = point.x;
            vpt.y = point.y;
            marker.points.push_back(vpt);
        }
        markerArray.markers.push_back(marker);

        // Create markers for start and end
        if (!path.empty()) {
            visualization_msgs::Marker marker{};

            marker.header.frame_id = "map";
            marker.ns = "mower_map_service_lines";
            marker.id = static_cast<int>(markerArray.markers.size());
            marker.frame_locked = true;
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.color = colors[0];
            marker.pose.position.x = path.front().x;
            marker.pose.position.y = path.front().y;
            marker.scale.x = 0.1;
            marker.scale.y = marker.scale.z = 0.1;
            markerArray.markers.push_back(marker);
        }
    }


    // keep track of the color used last, so that we can use a new one for each path
    uint32_t cidx = 0;

    // Walk through the paths we send to the navigation stack
    for (auto &path: planning_result.paths) {
        // Each group gets a single line strip as marker
        visualization_msgs::Marker marker;

        marker.header.frame_id = "map";
        marker.ns = "mower_map_service_lines";
        marker.id = static_cast<int>(markerArray.markers.size());
        marker.frame_locked = true;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.color = colors[cidx];
        marker.pose.orientation.w = 1;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.02;

        // Add the points to the line strip
        for (auto &point: path.path.poses) {

            geometry_msgs::Point vpt;
            vpt.x = point.pose.position.x;
            vpt.y = point.pose.position.y;
            marker.points.push_back(vpt);
        }
        markerArray.markers.push_back(marker);

        // Create markers for start
        if (!path.path.poses.empty()) {
            visualization_msgs::Marker marker;

            marker.header.frame_id = "map";
            marker.ns = "mower_map_service_lines";
            marker.id = static_cast<int>(markerArray.markers.size());
            marker.frame_locked = true;
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.color = colors[cidx];
            marker.pose = path.path.poses.front().pose;
            marker.scale.x = 0.2;
            marker.scale.y = marker.scale.z = 0.05;
            markerArray.markers.push_back(marker);
        }

        // New color for a new path
        cidx = (cidx + 1) % colors.size();
    }
}

// Detect pairs of sharp cusps (omega shapes) in generated paths and replace
// the small opposing-cusps region with a circular loop to avoid very steep turns.
void fixOmegaShapes(std::vector<slic3r_coverage_planner::Path> &paths,
                    double max_pair_distance,
                    double min_loop_radius,
                    int loop_points,
                    double cusp_angle_deg,
                    double approach_angle_deg,
                    std::vector<geometry_msgs::Point> &omega_points,
                    std::vector<geometry_msgs::Point> &candidate_points,
                    std::vector<geometry_msgs::Point> &cone_lines) {
    const double COS_ANGLE_THRESHOLD = std::cos(cusp_angle_deg * M_PI / 180.0);
    const double APPROACH_COS_THRESHOLD = std::cos(approach_angle_deg * M_PI / 180.0);

    for (auto &path: paths) {
        auto &poses = path.path.poses;
        if (poses.size() < 5) continue;

        double dx_end = poses.back().pose.position.x - poses.front().pose.position.x;
        double dy_end = poses.back().pose.position.y - poses.front().pose.position.y;
        double end_dist = std::hypot(dx_end, dy_end);
        ROS_INFO("Path size=%zu, first=(%f,%f), last=(%f,%f), end_dist=%f", poses.size(),
             poses.front().pose.position.x, poses.front().pose.position.y,
             poses.back().pose.position.x, poses.back().pose.position.y,
             end_dist);

        // We'll detect cusps and allow multiple replacements per path by
        // recomputing cusp candidates after each replacement.
        std::vector<geometry_msgs::Point> local_candidate_points;
        std::vector<geometry_msgs::Point> local_cone_lines;
        std::vector<geometry_msgs::Point> local_omega_points;

        while (true) {
            std::vector<int> cusp_indices;
            size_t n = poses.size();
            bool closed = false;
            {
                double dx = poses.front().pose.position.x - poses.back().pose.position.x;
                double dy = poses.front().pose.position.y - poses.back().pose.position.y;
                if (std::hypot(dx, dy) < 1e-6) closed = true;
            }

            // allow treating endpoints as circular neighbors if they are close (< 10cm)
            bool end_near = end_dist < 0.1; // 10 cm

            for (size_t idx = 0; idx < n; ++idx) {
                size_t i = idx;
                // determine previous index: allow wrapping if closed or endpoints are near
                size_t prev;
                if (i == 0) {
                    if (closed || end_near) prev = n - 1;
                    else continue; // skip first point for open paths
                } else {
                    prev = i - 1;
                }
                // determine next index similarly
                size_t next;
                if (i + 1 == n) {
                    if (closed || end_near) next = 0;
                    else continue; // skip last point for open paths
                } else {
                    next = i + 1;
                }

                // Vector from previous point -> current cusp candidate
                double ax = poses[i].pose.position.x - poses[prev].pose.position.x;
                double ay = poses[i].pose.position.y - poses[prev].pose.position.y;
                // Vector from current cusp candidate -> next point
                double bx = poses[next].pose.position.x - poses[i].pose.position.x;
                double by = poses[next].pose.position.y - poses[i].pose.position.y;

                // lengths of the two incident segments
                double la = std::hypot(ax, ay);
                double lb = std::hypot(bx, by);
                // Dismiss degenerate segments (very small length)
                if (la < 1e-9 || lb < 1e-9) {
                    ROS_INFO("Dismissing very small segment=%f %f", la, lb);
                    continue;
                }

                // Compute cosine of angle between the two incident segments.
                // dot is close to -1 for a sharp opposite direction (omega), and +1 for straight line.
                double dot = (ax * bx + ay * by) / (la * lb);

                // Selection condition: the angle at this point must be sufficiently sharp to be a 'cusp'.
                // We use COS_ANGLE_THRESHOLD = cos(130deg). If dot < COS_ANGLE_THRESHOLD, the angle
                // between incoming and outgoing vectors is > 130deg and we consider this a cusp candidate.
                if (dot < COS_ANGLE_THRESHOLD) {
                    ROS_INFO("Candidate at =%f %f, dot=%f",
                             poses[i].pose.position.x,
                             poses[i].pose.position.y,
                             dot);

                    // register cusp index for later pairing
                    cusp_indices.push_back((int)i);

                    // register candidate point (for visualization) locally
                    geometry_msgs::Point cp;
                    cp.x = poses[i].pose.position.x;
                    cp.y = poses[i].pose.position.y;
                    cp.z = poses[i].pose.position.z;
                    local_candidate_points.push_back(cp);

                    // Build a visualization cone showing the approach direction. This cone uses the
                    // approach vector (from previous point to the cusp) and an angular aperture
                    // defined by acos(APPROACH_COS_THRESHOLD).
                    double aix = poses[i].pose.position.x - poses[prev].pose.position.x;
                    double aiy = poses[i].pose.position.y - poses[prev].pose.position.y;
                    double approach_len = std::hypot(aix, aiy);
                    if (approach_len > 1e-9) {
                        double ux = aix / approach_len;
                        double uy = aiy / approach_len;
                        double ang = std::acos(APPROACH_COS_THRESHOLD); // half-aperture of approach cone
                        double ca = std::cos(ang);
                        double sa = std::sin(ang);
                        // rotate (ux,uy) by +ang
                        double rx1 = ux * ca - uy * sa;
                        double ry1 = ux * sa + uy * ca;
                        // rotate by -ang
                        double rx2 = ux * ca + uy * sa;
                        double ry2 = -ux * sa + uy * ca;
                        double cone_len = max_pair_distance; // visualize up to pairing distance
                        geometry_msgs::Point p1; p1.x = cp.x + rx1 * cone_len; p1.y = cp.y + ry1 * cone_len; p1.z = cp.z;
                        geometry_msgs::Point p2; p2.x = cp.x + rx2 * cone_len; p2.y = cp.y + ry2 * cone_len; p2.z = cp.z;
                        // add two lines: cusp->p1 and cusp->p2 (for visualization only)
                        local_cone_lines.push_back(cp);
                        local_cone_lines.push_back(p1);
                        local_cone_lines.push_back(cp);
                        local_cone_lines.push_back(p2);
                    }
                }
            }

            // Try to find and replace pairs until none remain for the current poses.
            bool replaced = false;
            for (size_t ci = 0; ci < cusp_indices.size() && !replaced; ++ci) {
                for (size_t cj = ci + 1; cj < cusp_indices.size() && !replaced; ++cj) {
                    int i = cusp_indices[ci];
                    int j = cusp_indices[cj];

                    double dx = poses[j].pose.position.x - poses[i].pose.position.x;
                    double dy = poses[j].pose.position.y - poses[i].pose.position.y;
                    double dist = std::hypot(dx, dy);

                    ROS_INFO("Cusps at =%f %f %f %f, dist=%f",
                        poses[i].pose.position.x,
                        poses[i].pose.position.y,
                        poses[j].pose.position.x,
                        poses[j].pose.position.y,
                        dist
                    );

                    if (dist > max_pair_distance) {
                        ROS_INFO("  Too far (dist=%f <= %f)", dist, max_pair_distance);
                        continue;
                    }
                    else {
                        ROS_INFO("  Not too far (dist=%f <= %f)", dist, max_pair_distance);
                    }

                    // check that the approach to cusp i is directed toward cusp j
                    size_t prev_i = (i == 0) ? (n - 1) : (i - 1);
                    double aix = poses[i].pose.position.x - poses[prev_i].pose.position.x;
                    double aiy = poses[i].pose.position.y - poses[prev_i].pose.position.y;
                    double tijx = poses[j].pose.position.x - poses[i].pose.position.x;
                    double tijy = poses[j].pose.position.y - poses[i].pose.position.y;
                    double na = std::hypot(aix, aiy);
                    double nb = std::hypot(tijx, tijy);
                    if (na < 1e-9 || nb < 1e-9) continue;
                    double approach_dot = (aix * tijx + aiy * tijy) / (na * nb);
                    if (approach_dot < APPROACH_COS_THRESHOLD) continue;

                    // Replace subpath between i and j (inclusive) by reordering existing points
                    std::vector<geometry_msgs::PoseStamped> newposes;
                    for (int k = 0; k <= i - 1; ++k) newposes.push_back(poses[k]);
                    newposes.push_back(poses[i]);
                    newposes.push_back(poses[j]);
                    for (int k = (int)j - 1; k >= i + 1; --k) {
                        newposes.push_back(poses[k]);
                    }
                    newposes.push_back(poses[i]);
                    newposes.push_back(poses[j]);
                    for (size_t k = j + 1; k < poses.size(); ++k) newposes.push_back(poses[k]);

                    if (newposes.size() >= 2) {
                        for (size_t k = 0; k + 1 < newposes.size(); ++k) {
                            double dxn = newposes[k + 1].pose.position.x - newposes[k].pose.position.x;
                            double dyn = newposes[k + 1].pose.position.y - newposes[k].pose.position.y;
                            double orient = std::atan2(dyn, dxn);
                            tf2::Quaternion q(0.0, 0.0, orient);
                            newposes[k].pose.orientation = tf2::toMsg(q);
                        }
                        newposes.back().pose.orientation = newposes[newposes.size() - 2].pose.orientation;
                    }

                    // mark both cusp points for visualization (locally)
                    geometry_msgs::Point pa; pa.x = poses[i].pose.position.x; pa.y = poses[i].pose.position.y; pa.z = poses[i].pose.position.z;
                    geometry_msgs::Point pb; pb.x = poses[j].pose.position.x; pb.y = poses[j].pose.position.y; pb.z = poses[j].pose.position.z;
                    local_omega_points.push_back(pa);
                    local_omega_points.push_back(pb);

                    poses.swap(newposes);
                    replaced = true;
                }
            }

            // If we performed a replacement, recompute cusps on the updated poses to find further pairs.
            if (!replaced) {
                // Move locally collected visualization points into the global accumulators
                for (auto &p: local_candidate_points) candidate_points.push_back(p);
                for (auto &p: local_cone_lines) cone_lines.push_back(p);
                for (auto &p: local_omega_points) omega_points.push_back(p);
                break;
            }
            // else: loop again to detect further pairs on the modified poses
        }
    }
}

void traverse_from_left(std::vector<PerimeterGeneratorLoop> &contours, std::vector<Polygons> &line_groups) {
    for (auto &contour: contours) {
        if (contour.children.empty()) {
            line_groups.push_back(Polygons());
        } else {
            traverse_from_left(contour.children, line_groups);
        }
        line_groups.back().push_back(contour.polygon);
    }
}

void traverse_from_right(std::vector<PerimeterGeneratorLoop> &contours, std::vector<Polygons> &line_groups) {
    for (auto &contour: boost::adaptors::reverse(contours)) {
        if (contour.children.empty()) {
            line_groups.push_back(Polygons());
        } else {
            traverse_from_right(contour.children, line_groups);
        }
        line_groups.back().push_back(contour.polygon);
    }
}

slic3r_coverage_planner::Path determinePathForOutline(std_msgs::Header &header, Slic3r::Polygon &outline_poly, Slic3r::Polygons &group, bool isObstacle, Point *areaLastPoint) {
    slic3r_coverage_planner::Path path;
    path.is_outline = true;
    path.path.header = header;

    Point lastPoint;
    bool is_first_point = true;
    for (int i = 0; i < group.size(); i++) {
        auto points = group[i].equally_spaced_points(scale_(0.1));
        if (points.size() < 2) {
            ROS_INFO("Skipping single dot");
            continue;
        }
        ROS_INFO_STREAM("Got " << points.size() << " points");

        if (!is_first_point) {
            // Find a good transition point between the loops.
            // It should be close to the last split point, so that we don't need to traverse a lot.

            // Find the point in the current poly which is closest to the last point of the last group
            // (which is the next inner poly from this point of view).
            const auto last_x = unscale(lastPoint.x);
            const auto last_y = unscale(lastPoint.y);
            double min_distance = INFINITY;
            int closest_idx = 0;
            for (int idx = 0; idx < points.size(); ++idx) {
                const auto &pt = points[idx];
                const auto pt_x = unscale(pt.x);
                const auto pt_y = unscale(pt.y);
                double distance = sqrt((pt_x - last_x) * (pt_x - last_x) + (pt_y - last_y) * (pt_y - last_y));
                if (distance < min_distance) {
                    min_distance = distance;
                    closest_idx = idx;
                }
            }

            // In order to smooth the transition we skip some points (think spiral movement of the mower).
            // Check, that the skip did not break the path (cross the outer poly during transition).
            // If it's fine, use the smoothed path, otherwise use the shortest point to split.
            int smooth_transition_idx = (closest_idx + 3) % points.size();

            const Polygon *next_outer_poly;
            if (i < group.size() - 1) {
                next_outer_poly = &group[i + 1];
            } else {
                // we are in the outermost line, use outline for collision check
                next_outer_poly = &outline_poly;
            }
            Line connection(points[smooth_transition_idx], lastPoint);
            Point intersection_pt{};
            if (next_outer_poly->intersection(connection, &intersection_pt)) {
                // intersection, we need to transition at closest point
                smooth_transition_idx = closest_idx;
            }

            if (smooth_transition_idx > 0) {
                std::rotate(points.begin(), points.begin() + smooth_transition_idx, points.end());
            }
        }

        for (auto &pt: points) {
            if (is_first_point) {
                lastPoint = pt;
                is_first_point = false;
                continue;
            }

            // calculate pose for "lastPoint" pointing to current point

            // Direction for obstacle needs to be inversed compared to area outline, because we will reverse the point order later.
            auto dir = isObstacle ? lastPoint - pt : pt - lastPoint;

            double orientation = atan2(dir.y, dir.x);
            tf2::Quaternion q(0.0, 0.0, orientation);

            geometry_msgs::PoseStamped pose;
            pose.header = header;
            pose.pose.orientation = tf2::toMsg(q);
            pose.pose.position.x = unscale(lastPoint.x);
            pose.pose.position.y = unscale(lastPoint.y);
            pose.pose.position.z = 0;
            path.path.poses.push_back(pose);
            lastPoint = pt;
        }
    }

    if (is_first_point) {
        // there wasn't any usable point, so return the empty path
        return path;
    }

    // finally, we add the final pose for "lastPoint" with the same orientation as the last pose
    geometry_msgs::PoseStamped pose;
    pose.header = header;
    pose.pose.orientation = path.path.poses.back().pose.orientation;
    pose.pose.position.x = unscale(lastPoint.x);
    pose.pose.position.y = unscale(lastPoint.y);
    pose.pose.position.z = 0;
    path.path.poses.push_back(pose);

    if (areaLastPoint != nullptr) {
        *areaLastPoint = lastPoint;
    }

    return path;
}

bool planPath(slic3r_coverage_planner::PlanPathRequest &req, slic3r_coverage_planner::PlanPathResponse &res) {
    Slic3r::Polygon outline_poly;
    for (auto &pt: req.outline.points) {
        outline_poly.points.push_back(Point(scale_(pt.x), scale_(pt.y)));
    }

    outline_poly.make_counter_clockwise();

    // This ExPolygon contains our input area with holes.
    Slic3r::ExPolygon expoly(outline_poly);

    for (auto &hole: req.holes) {
        Slic3r::Polygon hole_poly;
        for (auto &pt: hole.points) {
            hole_poly.points.push_back(Point(scale_(pt.x), scale_(pt.y)));
        }
        hole_poly.make_clockwise();

        if (intersection(outline_poly, hole_poly).empty()) continue;

        expoly.holes.push_back(hole_poly);
    }





    // Results are stored here
    std::vector<Polygons> area_outlines;
    Polylines fill_lines;
    std::vector<Polygons> obstacle_outlines;


    coord_t distance = scale_(req.distance);
    coord_t outer_distance = scale_(req.outer_offset);

    // detect how many perimeters must be generated for this island
    int loops = req.outline_count;

    ROS_INFO_STREAM("generating " << loops << " outlines");

    const int loop_number = loops - 1;  // 0-indexed loops
    const int inner_loop_number = loop_number - req.outline_overlap_count;


    Polygons gaps;

    Polygons last = expoly;
    Polygons inner = last;
    if (loop_number >= 0) {  // no loops = -1

        std::vector<PerimeterGeneratorLoops> contours(loop_number + 1);    // depth => loops
        std::vector<PerimeterGeneratorLoops> holes(loop_number + 1);       // depth => loops

        for (int i = 0; i <= loop_number; ++i) {  // outer loop is 0
            Polygons offsets;

            if (i == 0) {
                offsets = offset(
                        last,
                        -outer_distance
                );
            } else {
                offsets = offset(
                        last,
                        -distance
                );
            }

            if (offsets.empty()) break;


            last = offsets;
            if (i <= inner_loop_number) {
                inner = last;
            }

            for (Polygons::const_iterator polygon = offsets.begin(); polygon != offsets.end(); ++polygon) {
                PerimeterGeneratorLoop loop(*polygon, i);
                loop.is_contour = polygon->is_counter_clockwise();
                if (loop.is_contour) {
                    contours[i].push_back(loop);
                } else {
                    holes[i].push_back(loop);
                }
            }
        }

        // nest loops: holes first
        for (int d = 0; d <= loop_number; ++d) {
            PerimeterGeneratorLoops &holes_d = holes[d];

            // loop through all holes having depth == d
            for (int i = 0; i < (int) holes_d.size(); ++i) {
                const PerimeterGeneratorLoop &loop = holes_d[i];

                // find the hole loop that contains this one, if any
                for (int t = d + 1; t <= loop_number; ++t) {
                    for (int j = 0; j < (int) holes[t].size(); ++j) {
                        PerimeterGeneratorLoop &candidate_parent = holes[t][j];
                        if (candidate_parent.polygon.contains(loop.polygon.first_point())) {
                            candidate_parent.children.push_back(loop);
                            holes_d.erase(holes_d.begin() + i);
                            --i;
                            goto NEXT_LOOP;
                        }
                    }
                }

                NEXT_LOOP:;
            }
        }

        // nest contour loops
        for (int d = loop_number; d >= 1; --d) {
            PerimeterGeneratorLoops &contours_d = contours[d];

            // loop through all contours having depth == d
            for (int i = 0; i < (int) contours_d.size(); ++i) {
                const PerimeterGeneratorLoop &loop = contours_d[i];

                // find the contour loop that contains it
                for (int t = d - 1; t >= 0; --t) {
                    for (size_t j = 0; j < contours[t].size(); ++j) {
                        PerimeterGeneratorLoop &candidate_parent = contours[t][j];
                        if (candidate_parent.polygon.contains(loop.polygon.first_point())) {
                            candidate_parent.children.push_back(loop);
                            contours_d.erase(contours_d.begin() + i);
                            --i;
                            goto NEXT_CONTOUR;
                        }
                    }
                }

                NEXT_CONTOUR:;
            }
        }

        traverse_from_right(contours[0], area_outlines);
        for (auto &hole: holes) {
            traverse_from_left(hole, obstacle_outlines);
        }

        for (auto &obstacle_group: obstacle_outlines) {
            for (auto &poly: obstacle_group) {
                std::reverse(poly.points.begin(), poly.points.end());
            }
        }
    }


    ExPolygons expp = union_ex(inner);


    // Go through the innermost poly and create the fill path using a Fill object
    for (auto &poly: expp) {
        Slic3r::Surface surface(Slic3r::SurfaceType::stBottom, poly);


        Slic3r::Fill *fill;
        if (req.fill_type == slic3r_coverage_planner::PlanPathRequest::FILL_LINEAR) {
            fill = new Slic3r::FillRectilinear();
        } else {
            fill = new Slic3r::FillConcentric();
        }
        fill->link_max_length = scale_(1.0);
        fill->angle = req.angle;
        fill->z = scale_(1.0);
        fill->endpoints_overlap = 0;
        fill->density = 1.0;
        fill->dont_connect = false;
        fill->dont_adjust = false;
        fill->min_spacing = req.distance;
        fill->complete = false;
        fill->link_max_length = 0;

        ROS_INFO_STREAM("Starting Fill. Poly size:" << surface.expolygon.contour.points.size());

        Slic3r::Polylines lines = fill->fill_surface(surface);
        append_to(fill_lines, lines);
        delete fill;
        fill = nullptr;

        ROS_INFO_STREAM("Fill Complete. Polyline count: " << lines.size());
        for (int i = 0; i < lines.size(); i++) {
            ROS_INFO_STREAM("Polyline " << i << " has point count: " << lines[i].points.size());
        }
    }


    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "map";
    header.seq = 0;

    /**
     * Some postprocessing is done here. Until now we just have polygons (just points), but the ROS
     * navigation stack requires an orientation for each of those points as well.
     *
     * In order to achieve this, we split the polygon at some point to make it into a line with start and end.
     * Then we can calculate the orientation at each point by looking at the connection line between two points.
     */

    Point areaLastPoint;
    for (auto &group: area_outlines) {
        auto path = determinePathForOutline(header, outline_poly, group, false, &areaLastPoint);
        if (!path.path.poses.empty()) {
            res.paths.push_back(path);
        }
    }

    // The order for 3d printing seems to be to sweep across the X and then up the Y axis
    // which is very inefficient for a mower. Order the holes by distance to the previous end-point instead.
    std::vector<Slic3r::Polygons> ordered_obstacle_outlines;
    if (obstacle_outlines.size() > 0) {
        // If no prev point set to the first point in first obstacle
        // Note: back() polygon is the first (outer) loop
        auto prev_point = area_outlines.size() > 0 ? &areaLastPoint :
            &obstacle_outlines.front().back().points.front();

        while (obstacle_outlines.size()) {
            // Sort be desc distance then pop closest outline from the back of the vector
            std::sort(obstacle_outlines.begin(), obstacle_outlines.end(),
                      [prev_point](Slic3r::Polygons &a, Slic3r::Polygons &b) {
                          // Note: back() polygon is the first (outer) loop
                          auto a_firstPoint = a.back().points.front();
                          double distance_a = sqrt(
                                  (a_firstPoint.x - prev_point->x) * (a_firstPoint.x - prev_point->x) +
                                  (a_firstPoint.y - prev_point->y) * (a_firstPoint.y - prev_point->y)
                          );
                          auto b_firstPoint = b.back().points.front();
                          double distance_b = sqrt(
                                  (b_firstPoint.x - prev_point->x) * (b_firstPoint.x - prev_point->x) +
                                  (b_firstPoint.y - prev_point->y) * (b_firstPoint.y - prev_point->y)
                          );
                          return distance_a >= distance_b;
                      });
            ordered_obstacle_outlines.push_back(obstacle_outlines.back());
            obstacle_outlines.pop_back();
            // Note: front() polygon is the last (inner) loop
            prev_point = &ordered_obstacle_outlines.back().front().points.back();
        }
    }

    // At this point, the obstacles outlines are still "the wrong way" (i.e. inner first, then outer ...),
    // this is intentional, because then it's easier to find good traversal points.
    // In order to make the mower approach the obstacle, we will reverse the path later.
    for (auto &group: ordered_obstacle_outlines) {
        // Reverse here to make the mower approach the obstacle instead of starting close to the obstacle
        auto path = determinePathForOutline(header, outline_poly, group, true, nullptr);
        if (!path.path.poses.empty()) {
            std::reverse(path.path.poses.begin(), path.path.poses.end());
            res.paths.push_back(path);
        }
    }

    // Collect fill-line paths into a local vector so we can reorder them.
    std::vector<slic3r_coverage_planner::Path> fill_paths;
    for (int i = 0; i < fill_lines.size(); i++) {
        auto &line = fill_lines[i];
        slic3r_coverage_planner::Path path;
        path.is_outline = false;
        path.path.header = header;

        line.remove_duplicate_points();

        auto equally_spaced_points = line.equally_spaced_points(scale_(0.1));
        if (equally_spaced_points.size() < 2) {
            ROS_INFO("Skipping single dot");
            continue;
        }
        ROS_INFO_STREAM("Got " << equally_spaced_points.size() << " points");

        Point *lastPoint = nullptr;
        for (auto &pt: equally_spaced_points) {
            if (lastPoint == nullptr) {
                lastPoint = &pt;
                continue;
            }

            // calculate pose for "lastPoint" pointing to current point
            auto dir = pt - *lastPoint;
            double orientation = atan2(dir.y, dir.x);
            tf2::Quaternion q(0.0, 0.0, orientation);

            geometry_msgs::PoseStamped pose;
            pose.header = header;
            pose.pose.orientation = tf2::toMsg(q);
            pose.pose.position.x = unscale(lastPoint->x);
            pose.pose.position.y = unscale(lastPoint->y);
            pose.pose.position.z = 0;
            path.path.poses.push_back(pose);
            lastPoint = &pt;
        }

        // finally, we add the final pose for "lastPoint" with the same orientation as the last pose
        geometry_msgs::PoseStamped pose;
        pose.header = header;
        pose.pose.orientation = path.path.poses.back().pose.orientation;
        pose.pose.position.x = unscale(lastPoint->x);
        pose.pose.position.y = unscale(lastPoint->y);
        pose.pose.position.z = 0;
        path.path.poses.push_back(pose);

        fill_paths.push_back(path);
    }

    // Nearest-neighbour ordering with optional per-path reversal.
    //
    // Starting point: the endpoint of the last outline/obstacle path already in res.paths
    // (or the first fill path's front if no outlines were generated).
    // For each unvisited fill path we try both ends and pick the one closer to prev_endpoint.
    // When the back end is closer we reverse the path and flip every pose orientation by pi so
    // the robot travels in the right direction and arrives at the start with the correct heading.
    {
        double prev_x = 0.0, prev_y = 0.0;
        if (!res.paths.empty() && !res.paths.back().path.poses.empty()) {
            prev_x = res.paths.back().path.poses.back().pose.position.x;
            prev_y = res.paths.back().path.poses.back().pose.position.y;
        } else if (!fill_paths.empty() && !fill_paths.front().path.poses.empty()) {
            prev_x = fill_paths.front().path.poses.front().pose.position.x;
            prev_y = fill_paths.front().path.poses.front().pose.position.y;
        }

        std::vector<bool> visited(fill_paths.size(), false);
        for (size_t i = 0; i < fill_paths.size(); ++i) {
            double best_dist = std::numeric_limits<double>::infinity();
            int    best_idx  = -1;
            bool   best_rev  = false;

            for (size_t j = 0; j < fill_paths.size(); ++j) {
                if (visited[j]) continue;
                const auto &poses = fill_paths[j].path.poses;
                if (poses.empty()) continue;

                double dx_f = poses.front().pose.position.x - prev_x;
                double dy_f = poses.front().pose.position.y - prev_y;
                double dist_f = std::sqrt(dx_f * dx_f + dy_f * dy_f);

                double dx_b = poses.back().pose.position.x - prev_x;
                double dy_b = poses.back().pose.position.y - prev_y;
                double dist_b = std::sqrt(dx_b * dx_b + dy_b * dy_b);

                if (dist_f < best_dist) { best_dist = dist_f; best_idx = (int)j; best_rev = false; }
                if (dist_b < best_dist) { best_dist = dist_b; best_idx = (int)j; best_rev = true;  }
            }

            if (best_idx < 0) break;
            visited[best_idx] = true;

            auto &chosen = fill_paths[best_idx];
            if (best_rev) {
                std::reverse(chosen.path.poses.begin(), chosen.path.poses.end());
                // Flip every orientation by pi so the heading matches the reversed travel direction.
                // The btQuaternion 3-arg ctor is (yaw_Y, pitch_X, roll_Z) where roll_Z is the 2D yaw.
                for (auto &ps : chosen.path.poses) {
                    tf2::Quaternion q(ps.pose.orientation.x, ps.pose.orientation.y,
                                      ps.pose.orientation.z, ps.pose.orientation.w);
                    double yaw = 2.0 * std::atan2(q.z(), q.w());
                    tf2::Quaternion q_new(0.0, 0.0, yaw + M_PI);
                    ps.pose.orientation = tf2::toMsg(q_new);
                }
                ROS_INFO_STREAM("Fill path " << best_idx << " reversed for shorter transition (dist=" << best_dist << "m).");
            }

            if (!chosen.path.poses.empty()) {
                prev_x = chosen.path.poses.back().pose.position.x;
                prev_y = chosen.path.poses.back().pose.position.y;
            }

            res.paths.push_back(chosen);
        }
    }

    // Post-process generated paths to remove omega-shaped opposing cusps by inserting loops
    std::vector<geometry_msgs::Point> omega_points;
    std::vector<geometry_msgs::Point> candidate_points;
    std::vector<geometry_msgs::Point> cone_lines;
    if (omega_detection_enabled) {
        fixOmegaShapes(res.paths, omega_max_pair_distance, 0.05, 16,
                       omega_cusp_angle_deg, omega_approach_angle_deg,
                       omega_points, candidate_points, cone_lines);
    }

    if (visualize_plan) {
        visualization_msgs::MarkerArray arr;
        {
            visualization_msgs::Marker marker;

            marker.header.frame_id = "map";
            marker.ns = "mower_map_service_lines";
            marker.id = -1;
            marker.frame_locked = true;
            marker.action = visualization_msgs::Marker::DELETEALL;
            arr.markers.push_back(marker);
        }
        createMarkers(req, res, arr);

        /* mark candidate to omega fixing */
        if (!candidate_points.empty()) {
            visualization_msgs::Marker cand_marker;
            cand_marker.header.frame_id = "map";
            cand_marker.ns = "omega_candidates";
            cand_marker.id = static_cast<int>(arr.markers.size());
            cand_marker.frame_locked = true;
            cand_marker.action = visualization_msgs::Marker::ADD;
            cand_marker.type = visualization_msgs::Marker::SPHERE_LIST;
            std_msgs::ColorRGBA cc; cc.r = 1.0; cc.g = 1.0; cc.b = 0.0; cc.a = 1.0;
            cand_marker.color = cc;
            cand_marker.pose.orientation.w = 1.0;
            cand_marker.scale.x = cand_marker.scale.y = cand_marker.scale.z = 0.06;
            for (auto &p: candidate_points) cand_marker.points.push_back(p);
            arr.markers.push_back(cand_marker);
        }

        if (!cone_lines.empty()) {
            visualization_msgs::Marker cone_marker;
            cone_marker.header.frame_id = "map";
            cone_marker.ns = "omega_cones";
            cone_marker.id = static_cast<int>(arr.markers.size());
            cone_marker.frame_locked = true;
            cone_marker.action = visualization_msgs::Marker::ADD;
            cone_marker.type = visualization_msgs::Marker::LINE_LIST;
            std_msgs::ColorRGBA cc; cc.r = 1.0; cc.g = 1.0; cc.b = 1.0; cc.a = 1.0;
            cone_marker.color = cc;
            cone_marker.pose.orientation.w = 1.0;
            cone_marker.scale.x = 0.02;
            for (auto &p: cone_lines) cone_marker.points.push_back(p);
            arr.markers.push_back(cone_marker);
        }

        if (!omega_points.empty()) {
            visualization_msgs::Marker omega_marker;
            omega_marker.header.frame_id = "map";
            omega_marker.ns = "omega_points";
            omega_marker.id = static_cast<int>(arr.markers.size());
            omega_marker.frame_locked = true;
            omega_marker.action = visualization_msgs::Marker::ADD;
            omega_marker.type = visualization_msgs::Marker::SPHERE_LIST;
            std_msgs::ColorRGBA c; c.r = 1.0; c.g = 0.0; c.b = 1.0; c.a = 1.0;
            omega_marker.color = c;
            omega_marker.pose.orientation.w = 1.0;
            omega_marker.scale.x = omega_marker.scale.y = omega_marker.scale.z = 0.08;
            for (auto &p: omega_points) omega_marker.points.push_back(p);
            arr.markers.push_back(omega_marker);
        }

        marker_array_publisher.publish(arr);
    }


    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "slic3r_coverage_planner");

    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    visualize_plan = paramNh.param("visualize_plan", true);
    omega_detection_enabled = paramNh.param("omega_detection_enabled", true);
    omega_max_pair_distance = paramNh.param("omega_max_pair_distance", 0.55);
    omega_cusp_angle_deg = paramNh.param("omega_cusp_angle_deg", 130.0);
    omega_approach_angle_deg = paramNh.param("omega_approach_angle_deg", 20.0);

    if (visualize_plan) {
        marker_array_publisher = n.advertise<visualization_msgs::MarkerArray>(
                "slic3r_coverage_planner/path_marker_array", 100, true);
    }

    ros::ServiceServer plan_path_srv = n.advertiseService("slic3r_coverage_planner/plan_path", planPath);

    ros::spin();
    return 0;
}
