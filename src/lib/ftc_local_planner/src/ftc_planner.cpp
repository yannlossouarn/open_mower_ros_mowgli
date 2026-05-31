
#include <ftc_local_planner/ftc_planner.h>

#include <pluginlib/class_list_macros.h>
#include "mbf_msgs/ExePathAction.h"

PLUGINLIB_EXPORT_CLASS(ftc_local_planner::FTCPlanner, mbf_costmap_core::CostmapController)

#define RET_SUCCESS 0
#define RET_COLLISION 104
#define RET_BLOCKED 109

namespace ftc_local_planner
{

    FTCPlanner::FTCPlanner()
    {
    }

    void FTCPlanner::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
    {
        ros::NodeHandle private_nh("~/" + name);

        progress_server = private_nh.advertiseService(
            "planner_get_progress", &FTCPlanner::getProgress, this);

        global_point_pub = private_nh.advertise<geometry_msgs::PoseStamped>("global_point", 1);
        global_plan_pub = private_nh.advertise<nav_msgs::Path>("global_plan", 1, true);
        obstacle_marker_pub = private_nh.advertise<visualization_msgs::Marker>("costmap_marker", 10);

        costmap = costmap_ros;
        costmap_map_ = costmap->getCostmap();
        tf_buffer = tf;

        // Parameter for dynamic reconfigure
        reconfig_server = new dynamic_reconfigure::Server<FTCPlannerConfig>(private_nh);
        dynamic_reconfigure::Server<FTCPlannerConfig>::CallbackType cb = boost::bind(&FTCPlanner::reconfigureCB, this,
                                                                                     _1, _2);
        reconfig_server->setCallback(cb);

        current_state = PRE_ROTATE;

        // PID Debugging topic
        if (config.debug_pid)
        {
            pubPid = private_nh.advertise<ftc_local_planner::PID>("debug_pid", 1, true);
        }

        // Recovery behavior initialization
        failure_detector_.setBufferLength(std::round(config.oscillation_recovery_min_duration * 10));

        // Shock and slip detection subscribers
        ros::NodeHandle nh;
        imu_sub_ = nh.subscribe("/ll/imu/data_raw", 10, &FTCPlanner::onImu, this,
                                ros::TransportHints().tcpNoDelay(true));
        measured_twist_sub_ = nh.subscribe("/ll/diff_drive/measured_twist", 10,
                                           &FTCPlanner::onMeasuredTwist, this,
                                           ros::TransportHints().tcpNoDelay(true));
        xb_pose_sub_ = nh.subscribe("/xbot_positioning/xb_pose", 10, &FTCPlanner::onXbPose, this,
                                    ros::TransportHints().tcpNoDelay(true));
        // Mowing rotor RPM + blade-enabled feedback for the rotor-load throttle.
        mower_status_sub_ = nh.subscribe("/ll/mower_status", 10, &FTCPlanner::onMowerStatus, this,
                                         ros::TransportHints().tcpNoDelay(true));

        shock_flag_.store(false);
        slip_window_active_ = false;
        stall_window_active_ = false;

        ROS_INFO("FTCLocalPlannerROS: Version 2 Init.");
    }

    void FTCPlanner::reconfigureCB(FTCPlannerConfig &c, uint32_t level)
    {
        if (c.restore_defaults)
        {
            reconfig_server->getConfigDefault(c);
            c.restore_defaults = false;
        }
        config = c;

        ROS_INFO_STREAM("FTCPlanner config: shock=" << (c.shock_detection_enabled ? "ENABLED" : "disabled")
            << " frontal_base=" << c.shock_frontal_base << " lateral_base=" << c.shock_lateral_base
            << " speed_factor=" << c.shock_speed_factor << " min_speed=" << c.shock_min_speed << "m/s"
            << " | slip=" << (c.slip_detection_enabled ? "ENABLED" : "disabled")
            << " wheel_min=" << c.slip_min_wheel_distance << "m ratio_thr=" << c.slip_ratio_threshold
            << " window=" << c.slip_detection_window << "s"
            << " | stall=" << (c.stall_detection_enabled ? "ENABLED" : "disabled")
            << " advance_min=" << c.stall_min_advance << "m window=" << c.stall_detection_window << "s");

        // just to be sure
        current_movement_speed = config.speed_slow;

        // set recovery behavior
        failure_detector_.setBufferLength(std::round(config.oscillation_recovery_min_duration * 10));
    }

    void FTCPlanner::onMowerStatus(const mower_msgs::Status::ConstPtr& msg)
    {
        mow_enabled_.store(msg->mow_enabled);
        rotor_rpm_.store(msg->mower_motor_rpm);
        mower_status_time_ = ros::Time::now();
    }

    // Throttle is active only while actually mowing (blade enabled) with a fresh status
    // signal — so transit/navigation paths (blade off) and stale-signal cases are unaffected.
    bool FTCPlanner::rotorThrottleActive()
    {
        if (!config.rotor_throttle_enabled) return false;
        if (!mow_enabled_.load()) return false;
        if (mower_status_time_.isZero()) return false;
        if ((ros::Time::now() - mower_status_time_).toSec() > config.rotor_signal_timeout) return false;
        return true;
    }

    // 1.0 = full speed (rotor at/above threshold), scaling down to rotor_throttle_min_factor
    // as the rotor bogs down under load.
    double FTCPlanner::rotorThrottleFactor()
    {
        if (!rotorThrottleActive() || config.rotor_rpm_threshold <= 0.0) return 1.0;
        double f = rotor_rpm_.load() / config.rotor_rpm_threshold;
        if (f > 1.0) f = 1.0;
        if (f < config.rotor_throttle_min_factor) f = config.rotor_throttle_min_factor;
        return f;
    }

    bool FTCPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        current_state = PRE_ROTATE;
        state_entered_time = ros::Time::now();
        is_crashed = false;

        global_plan = plan;
        current_index = 0;
        current_progress = 0.0;

        last_time = ros::Time::now();
        current_movement_speed = config.speed_slow;

        slip_window_active_ = false;
        stall_window_active_ = false;
        rotor_spinup_done_ = false;

        lat_error = 0.0;
        lon_error = 0.0;
        angle_error = 0.0;
        i_lon_error = 0.0;
        i_lat_error = 0.0;
        i_angle_error = 0.0;

        // Reset obstacle-aware rotation direction preferences.
        preferred_pre_rotate_sign_  = 0;
        preferred_post_rotate_sign_ = 0;

        nav_msgs::Path path;

        if (global_plan.size() > 2)
        {
            // duplicate last point
            global_plan.push_back(global_plan.back());
            // give second from last point last oriantation as the point before that
            global_plan[global_plan.size() - 2].pose.orientation = global_plan[global_plan.size() - 3].pose.orientation;
            path.header = plan.front().header;
            path.poses = plan;

            // Phase 3: choose the safer PRE_ROTATE direction up front.
            if (config.obstacle_aware_rotation) {
                geometry_msgs::PoseStamped robot_pose;
                if (costmap->getRobotPose(robot_pose)) {
                    tf2::Quaternion q_robot(robot_pose.pose.orientation.x, robot_pose.pose.orientation.y,
                                            robot_pose.pose.orientation.z, robot_pose.pose.orientation.w);
                    double robot_yaw = 2.0 * std::atan2(q_robot.z(), q_robot.w());

                    tf2::Quaternion q_first(global_plan[0].pose.orientation.x, global_plan[0].pose.orientation.y,
                                            global_plan[0].pose.orientation.z, global_plan[0].pose.orientation.w);
                    double pre_target_yaw = 2.0 * std::atan2(q_first.z(), q_first.w());
                    preferred_pre_rotate_sign_ = selectRotationDirection(robot_yaw, pre_target_yaw);
                }
            }
        }
        else
        {
            ROS_WARN_STREAM("FTCLocalPlannerROS: Global plan was too short. Need a minimum of 3 poses - Cancelling.");
            current_state = FINISHED;
            state_entered_time = ros::Time::now();
        }
        global_plan_pub.publish(path);

        ROS_INFO_STREAM("FTCLocalPlannerROS: Got new global plan with " << plan.size() << " points."
            << " pre_rotate_pref=" << preferred_pre_rotate_sign_);

        return true;
    }

    // -----------------------------------------------------------------------
    // Obstacle-aware rotation helpers  (Phase 3)
    // -----------------------------------------------------------------------

    double FTCPlanner::sweepFootprintCost(double cx, double cy, double angle)
    {
        std::vector<geometry_msgs::Point> footprint = costmap->getRobotFootprint();
        if (footprint.empty()) return 0.0;

        const double cos_a = std::cos(angle);
        const double sin_a = std::sin(angle);
        double max_cost = 0.0;

        for (const auto &fp : footprint) {
            double wx = cx + cos_a * fp.x - sin_a * fp.y;
            double wy = cy + sin_a * fp.x + cos_a * fp.y;
            unsigned int mx, my;
            if (costmap_map_->worldToMap(wx, wy, mx, my)) {
                unsigned char cost = costmap_map_->getCost(mx, my);
                if (cost > max_cost) max_cost = cost;
            } else {
                return static_cast<double>(costmap_2d::LETHAL_OBSTACLE);  // outside map
            }
        }
        return max_cost;
    }

    int FTCPlanner::selectRotationDirection(double robot_yaw, double target_yaw)
    {
        if (!config.obstacle_aware_rotation) return 0;

        // Normalise signed delta to [-π, π].
        double delta = target_yaw - robot_yaw;
        while (delta >  M_PI) delta -= 2.0 * M_PI;
        while (delta < -M_PI) delta += 2.0 * M_PI;

        geometry_msgs::PoseStamped robot_pose;
        if (!costmap->getRobotPose(robot_pose)) {
            return (delta >= 0) ? +1 : -1;  // fallback: shorter direction
        }
        const double cx = robot_pose.pose.position.x;
        const double cy = robot_pose.pose.position.y;

        double step = config.rotation_check_step_deg * M_PI / 180.0;
        if (step <= 0.0) step = 5.0 * M_PI / 180.0;

        // CCW arc: robot_yaw  →  robot_yaw + ccw_span  reaches target_yaw
        double ccw_span = (delta >= 0) ? delta : (2.0 * M_PI + delta);
        double max_cost_ccw = 0.0;
        for (double a = 0.0; a <= ccw_span; a += step) {
            double c = sweepFootprintCost(cx, cy, robot_yaw + a);
            if (c > max_cost_ccw) max_cost_ccw = c;
            if (c >= costmap_2d::LETHAL_OBSTACLE) break;  // no point checking further
        }

        // CW arc: robot_yaw  →  robot_yaw - cw_span  reaches target_yaw
        double cw_span = (delta <= 0) ? -delta : (2.0 * M_PI - delta);
        double max_cost_cw = 0.0;
        for (double a = 0.0; a <= cw_span; a += step) {
            double c = sweepFootprintCost(cx, cy, robot_yaw - a);
            if (c > max_cost_cw) max_cost_cw = c;
            if (c >= costmap_2d::LETHAL_OBSTACLE) break;
        }

        ROS_DEBUG_STREAM("FTCPlanner selectRotDir: delta=" << (delta * 180.0 / M_PI) << "deg"
            << " cost_ccw=" << max_cost_ccw << " cost_cw=" << max_cost_cw
            << " threshold=" << config.rotation_override_cost_threshold);

        // If both arcs have similar cost, keep the shorter/natural direction.
        if (std::abs(max_cost_ccw - max_cost_cw) < config.rotation_override_cost_threshold) {
            return (delta >= 0) ? +1 : -1;
        }
        return (max_cost_ccw <= max_cost_cw) ? +1 : -1;
    }

    FTCPlanner::~FTCPlanner()
    {
        if (reconfig_server != nullptr)
        {
            delete reconfig_server;
            reconfig_server = nullptr;
        }
    }

    // -----------------------------------------------------------------------
    // IMU shock detection callback
    // -----------------------------------------------------------------------
    void FTCPlanner::onImu(const sensor_msgs::Imu::ConstPtr& msg)
    {
        if (!config.shock_detection_enabled) return;
        if (current_state != FOLLOWING) return;
        if (current_movement_speed < config.shock_min_speed) return;

        const double eff_frontal = config.shock_frontal_base + config.shock_speed_factor * current_movement_speed;
        const double eff_lateral = config.shock_lateral_base + config.shock_speed_factor * current_movement_speed;

        const double ax = msg->linear_acceleration.x;
        const double ay = msg->linear_acceleration.y;
        ROS_INFO_STREAM_THROTTLE(10.0, "FTCPlanner IMU: ax=" << ax << " ay=" << ay
            << " | thr_frontal\u00b1" << eff_frontal << " thr_lateral\u00b1" << eff_lateral
            << " speed=" << current_movement_speed << "m/s");
        if (std::abs(ax) > eff_frontal || std::abs(ay) > eff_lateral)
        {
            if (!shock_flag_.load())
            {
                ROS_WARN_STREAM("FTCPlanner: Shock detected! ax=" << ax << " ay=" << ay
                    << " eff_frontal=" << eff_frontal << " eff_lateral=" << eff_lateral);
                shock_flag_.store(true);
                shock_flag_time_ = ros::Time::now();
            }
        }
    }

    // -----------------------------------------------------------------------
    // Wheel odometry accumulator for slip detection
    // -----------------------------------------------------------------------
    void FTCPlanner::onMeasuredTwist(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
        if (!config.slip_detection_enabled) return;
        if (current_state != FOLLOWING) return;
        if (current_movement_speed < config.shock_min_speed) return;

        const double vx = msg->twist.linear.x;
        const double dt = msg->header.stamp.isZero() ? 0.02 : 0.02; // nominal 50 Hz
        slip_wheel_distance_acc_ += std::abs(vx) * dt;
    }

    // -----------------------------------------------------------------------
    // GPS position accumulator — feeds both slip and stall detectors
    // -----------------------------------------------------------------------
    void FTCPlanner::onXbPose(const xbot_msgs::AbsolutePose::ConstPtr& msg)
    {
        last_xb_pose_ = *msg;
        const geometry_msgs::Point& pos = msg->pose.pose.position;
        const bool active = (current_state == FOLLOWING && current_movement_speed >= config.shock_min_speed);

        // --- Slip accumulator ---
        if (config.slip_detection_enabled)
        {
            if (!active)
            {
                slip_window_active_ = false;
            }
            else if (!slip_window_active_)
            {
                slip_window_active_ = true;
                slip_window_start_ = ros::Time::now();
                slip_wheel_distance_acc_ = 0.0;
                slip_gps_distance_acc_ = 0.0;
                slip_last_position_ = pos;
            }
            else
            {
                const double dx = pos.x - slip_last_position_.x;
                const double dy = pos.y - slip_last_position_.y;
                slip_gps_distance_acc_ += std::sqrt(dx * dx + dy * dy);
                slip_last_position_ = pos;
                ROS_INFO_STREAM_THROTTLE(5.0, "FTCPlanner slip window: elapsed="
                    << (ros::Time::now() - slip_window_start_).toSec() << "s"
                    << " wheel=" << slip_wheel_distance_acc_ << "m gps=" << slip_gps_distance_acc_ << "m"
                    << " ratio="
                    << (slip_wheel_distance_acc_ > 0.0 ? slip_gps_distance_acc_ / slip_wheel_distance_acc_ : -1.0)
                    << " gps_acc=" << msg->position_accuracy << "m");
            }
        }

        // --- Stall accumulator ---
        if (config.stall_detection_enabled)
        {
            const bool stall_active = (current_state == FOLLOWING && current_movement_speed >= config.stall_min_speed);
            if (!stall_active)
            {
                stall_window_active_ = false;
            }
            else if (!stall_window_active_)
            {
                stall_window_active_ = true;
                stall_window_start_ = ros::Time::now();
                stall_gps_acc_ = 0.0;
                stall_last_position_ = pos;
            }
            else
            {
                const double dx = pos.x - stall_last_position_.x;
                const double dy = pos.y - stall_last_position_.y;
                stall_gps_acc_ += std::sqrt(dx * dx + dy * dy);
                stall_last_position_ = pos;
            }
        }
    }

    // -----------------------------------------------------------------------
    // Mark the current robot footprint as lethal obstacle in the costmap
    // -----------------------------------------------------------------------
    void FTCPlanner::markObstacleAtCurrentPose()
    {
        geometry_msgs::PoseStamped robot_pose;
        if (!costmap->getRobotPose(robot_pose)) return;

        double wx = robot_pose.pose.position.x;
        double wy = robot_pose.pose.position.y;

        costmap_2d::Costmap2D* cm = costmap->getCostmap();
        unsigned int mx, my;
        if (cm->worldToMap(wx, wy, mx, my))
        {
            cm->setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
            ROS_INFO_STREAM("FTCPlanner: Marked obstacle at (" << wx << ", " << wy << ")");
        }
    }

    double FTCPlanner::distanceLookahead()
    {
        if (global_plan.size() < 2)
        {
            return 0;
        }
        Eigen::Quaternion<double> current_rot(current_control_point.linear());
        double lookahead_distance = 0.0;
        Eigen::Affine3d last_straight_point = current_control_point;
        Eigen::Affine3d current_point;
        for (uint32_t i = current_index + 1; i < global_plan.size(); i++)
        {
            tf2::fromMsg(global_plan[i].pose, current_point);

            // check, if direction is the same. if so, we add the distance
            Eigen::Quaternion<double> rot2(current_point.linear());

            if (lookahead_distance > config.speed_fast_threshold ||
                abs(rot2.angularDistance(current_rot)) > config.speed_fast_threshold_angle * (M_PI / 180.0))
            {
                break;
            }

            lookahead_distance += (current_point.translation() - last_straight_point.translation()).norm();
            last_straight_point = current_point;

        }

        return lookahead_distance;
    }

    uint32_t FTCPlanner::computeVelocityCommands(const geometry_msgs::PoseStamped &pose,
                                                 const geometry_msgs::TwistStamped &velocity,
                                                 geometry_msgs::TwistStamped &cmd_vel, std::string &message)
    {

        ros::Time now = ros::Time::now();
        double dt = now.toSec() - last_time.toSec();
        last_time = now;

        if (is_crashed)
        {
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.angular.z = 0;
            return RET_COLLISION;
        }

        if (current_state == FINISHED)
        {
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.angular.z = 0;
            return RET_SUCCESS;
        }

        // We're not crashed and not finished.
        // First, we update the control point if needed. This is needed since we need the local_control_point to calculate the next state.
        update_control_point(dt);
        // Then, update the planner state.
        auto new_planner_state = update_planner_state();
        if (new_planner_state != current_state)
        {
            ROS_INFO_STREAM("FTCLocalPlannerROS: Switching to state " << new_planner_state);
            state_entered_time = ros::Time::now();
            current_state = new_planner_state;
        }

        if (checkCollision(config.obstacle_lookahead))
        {
            ROS_WARN_STREAM("FTCPlanner: Costmap collision detected (lookahead=" << config.obstacle_lookahead
                << " segments) — triggering recovery");
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.angular.z = 0;
            is_crashed = true;
            markObstacleAtCurrentPose();
            return RET_BLOCKED;
        }

        // Check IMU shock flag (set asynchronously by onImu callback)
        if (config.shock_detection_enabled && shock_flag_.load())
        {
            shock_flag_.store(false);
            slip_window_active_ = false;
            stall_window_active_ = false;
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.angular.z = 0;
            is_crashed = true;
            markObstacleAtCurrentPose();
            ROS_WARN("FTCPlanner: Shock flag triggered is_crashed — requesting recovery");
            return RET_BLOCKED;
        }

        // Check wheel slip against GPS distance
        if (config.slip_detection_enabled && slip_window_active_)
        {
            const double elapsed = (ros::Time::now() - slip_window_start_).toSec();
            if (elapsed >= config.slip_detection_window)
            {
                if (slip_wheel_distance_acc_ >= config.slip_min_wheel_distance)
                {
                    const double ratio = (slip_wheel_distance_acc_ > 0.0)
                        ? slip_gps_distance_acc_ / slip_wheel_distance_acc_
                        : 1.0;
                    if (ratio < config.slip_ratio_threshold)
                    {
                        ROS_WARN_STREAM("FTCPlanner: Slip detected! wheel=" << slip_wheel_distance_acc_
                            << "m gps=" << slip_gps_distance_acc_ << "m ratio=" << ratio);
                        slip_window_active_ = false;
                        stall_window_active_ = false;
                        cmd_vel.twist.linear.x = 0;
                        cmd_vel.twist.angular.z = 0;
                        is_crashed = true;
                        markObstacleAtCurrentPose();
                        return RET_BLOCKED;
                    }
                }
                // Log window result and reset for next evaluation
                ROS_INFO_STREAM_THROTTLE(10.0, "FTCPlanner slip window closed OK: wheel=" << slip_wheel_distance_acc_
                    << "m gps=" << slip_gps_distance_acc_ << "m"
                    << " ratio=" << (slip_wheel_distance_acc_ > 0.0
                        ? slip_gps_distance_acc_ / slip_wheel_distance_acc_ : -1.0)
                    << " (thr=" << config.slip_ratio_threshold << ")");
                slip_window_start_ = ros::Time::now();
                slip_wheel_distance_acc_ = 0.0;
                slip_gps_distance_acc_ = 0.0;
            }
        }

        // Check position stall: GPS advance too small over the window despite commanded motion
        if (config.stall_detection_enabled && stall_window_active_)
        {
            const double elapsed = (ros::Time::now() - stall_window_start_).toSec();
            if (elapsed >= config.stall_detection_window)
            {
                if (stall_gps_acc_ < config.stall_min_advance)
                {
                    ROS_WARN_STREAM("FTCPlanner: Stall detected! GPS advance=" << stall_gps_acc_ << "m in " << elapsed
                                                                               << "s (min=" << config.stall_min_advance
                                                                               << "m) — requesting recovery");
                    stall_window_active_ = false;
                    slip_window_active_ = false;
                    cmd_vel.twist.linear.x = 0;
                    cmd_vel.twist.angular.z = 0;
                    is_crashed = true;
                    markObstacleAtCurrentPose();
                    return RET_BLOCKED;
                }
                ROS_INFO_STREAM_THROTTLE(10.0, "FTCPlanner stall window OK: GPS advance=" << stall_gps_acc_ << "m in "
                                                                                          << elapsed << "s (min="
                                                                                          << config.stall_min_advance
                                                                                          << "m)");
                stall_window_start_ = ros::Time::now();
                stall_gps_acc_ = 0.0;
            }
        }

        // Finally, we calculate the velocity commands.
        calculate_velocity_commands(dt, cmd_vel);

        if (is_crashed)
        {
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.angular.z = 0;
            return RET_COLLISION;
        }

        return RET_SUCCESS;
    }


    bool FTCPlanner::isGoalReached(double dist_tolerance, double angle_tolerance)
    {
        return current_state == FINISHED && !is_crashed;
    }

    bool FTCPlanner::cancel()
    {
        ROS_WARN_STREAM("FTCLocalPlannerROS: FTC planner was cancelled.");
        current_state = FINISHED;
        state_entered_time = ros::Time::now();
        return true;
    }

    FTCPlanner::PlannerState FTCPlanner::update_planner_state()
    {
        switch (current_state)
        {
        case PRE_ROTATE:
        {
            if (time_in_current_state() > config.goal_timeout)
            {
                ROS_ERROR_STREAM("FTCLocalPlannerROS: Error reaching goal. config.goal_timeout (" << config.goal_timeout << ") reached - Timeout in PRE_ROTATE phase.");
                is_crashed = true;
                return FINISHED;
            }
            if (abs(angle_error) * (180.0 / M_PI) < config.max_goal_angle_error)
            {
                ROS_INFO_STREAM("FTCLocalPlannerROS: PRE_ROTATE finished. Starting following");
                return FOLLOWING;
            }
        }
        break;
        case FOLLOWING:
        {
            double distance = local_control_point.translation().norm();
            // check for crash
            if (distance > config.max_follow_distance)
            {
                ROS_ERROR_STREAM("FTCLocalPlannerROS: Robot is far away from global plan. distance (" << distance << ") > config.max_follow_distance (" << config.max_follow_distance << ") It probably has crashed.");
                is_crashed = true;
                return FINISHED;
            }

            // check if we're done following
            if (current_index == global_plan.size() - 2)
            {
                ROS_INFO_STREAM("FTCLocalPlannerROS: switching planner to position mode");
                return WAITING_FOR_GOAL_APPROACH;
            }
        }
        break;
        case WAITING_FOR_GOAL_APPROACH:
        {
            double distance = local_control_point.translation().norm();
            if (time_in_current_state() > config.goal_timeout)
            {
                ROS_WARN_STREAM("FTCLocalPlannerROS: Could not reach goal position. config.goal_timeout (" << config.goal_timeout << ") reached - Attempting final rotation anyways.");
                return POST_ROTATE;
            }
            if (distance < config.max_goal_distance_error)
            {
                ROS_INFO_STREAM("FTCLocalPlannerROS: Reached goal position.");
                // Compute POST_ROTATE preferred direction now that we're at the goal position
                if (config.obstacle_aware_rotation)
                {
                    geometry_msgs::PoseStamped robot_pose;
                    if (costmap->getRobotPose(robot_pose))
                    {
                        tf2::Quaternion q_robot(robot_pose.pose.orientation.x,
                                                robot_pose.pose.orientation.y,
                                                robot_pose.pose.orientation.z,
                                                robot_pose.pose.orientation.w);
                        double robot_yaw = 2.0 * std::atan2(q_robot.z(), q_robot.w());
                        const auto &last_pose = global_plan.back();
                        tf2::Quaternion q_last(last_pose.pose.orientation.x,
                                               last_pose.pose.orientation.y,
                                               last_pose.pose.orientation.z,
                                               last_pose.pose.orientation.w);
                        double target_yaw = 2.0 * std::atan2(q_last.z(), q_last.w());
                        preferred_post_rotate_sign_ = selectRotationDirection(robot_yaw, target_yaw);
                    }
                }
                return POST_ROTATE;
            }
        }
        break;
        case POST_ROTATE:
        {
            if (time_in_current_state() > config.goal_timeout)
            {
                ROS_WARN_STREAM("FTCLocalPlannerROS: Could not reach goal rotation. config.goal_timeout (" << config.goal_timeout << ") reached");
                return FINISHED;
            }
            if (abs(angle_error) * (180.0 / M_PI) < config.max_goal_angle_error)
            {
                ROS_INFO_STREAM("FTCLocalPlannerROS: POST_ROTATE finished.");
                return FINISHED;
            }
        }
        break;
        case FINISHED:
        {
            // Nothing to do here
        }
        break;
        }

        return current_state;
    }

    void FTCPlanner::update_control_point(double dt)
    {

        switch (current_state)
        {
        case PRE_ROTATE:
            tf2::fromMsg(global_plan[0].pose, current_control_point);
            break;
        case FOLLOWING:
        {
            // Normal planner operation
            double straight_dist = distanceLookahead();
            double speed;
            if (straight_dist >= config.speed_fast_threshold)
            {
                speed = config.speed_fast;
            }
            else
            {
                speed = config.speed_slow;
            }

            // Rotor-load handling (mowing only; transit paths have the blade off so this is a no-op).
            // First hold forward motion until the rotor spins up to threshold, then throttle forward
            // speed proportionally if the rotor bogs down under load. The existing acceleration ramp
            // below smooths the deceleration/recovery.
            if (rotorThrottleActive())
            {
                if (!rotor_spinup_done_ && rotor_rpm_.load() < config.rotor_rpm_threshold &&
                    time_in_current_state() < config.rotor_spinup_timeout)
                {
                    speed = 0.0;  // wait for the blade to reach threshold (timeout fail-safe)
                }
                else
                {
                    rotor_spinup_done_ = true;
                    speed *= rotorThrottleFactor();
                }
            }

            if (speed > current_movement_speed)
            {
                // accelerate
                current_movement_speed += dt * config.acceleration;
                if (current_movement_speed > speed)
                    current_movement_speed = speed;
            }
            else if (speed < current_movement_speed)
            {
                // decelerate
                current_movement_speed -= dt * config.acceleration;
                if (current_movement_speed < speed)
                    current_movement_speed = speed;
            }

            double distance_to_move = dt * current_movement_speed;
            double angle_to_move = dt * config.speed_angular * (M_PI / 180.0);

            Eigen::Affine3d nextPose, currentPose;
            while (angle_to_move > 0 && distance_to_move > 0 && current_index < global_plan.size() - 2)
            {

                tf2::fromMsg(global_plan[current_index].pose, currentPose);
                tf2::fromMsg(global_plan[current_index + 1].pose, nextPose);

                double pose_distance = (nextPose.translation() - currentPose.translation()).norm();

                Eigen::Quaternion<double> current_rot(currentPose.linear());
                Eigen::Quaternion<double> next_rot(nextPose.linear());

                double pose_distance_angular = current_rot.angularDistance(next_rot);

                if (pose_distance <= 0.0)
                {
                    ROS_WARN_STREAM("FTCLocalPlannerROS: Skipping duplicate point in global plan.");
                    current_index++;
                    continue;
                }

                double remaining_distance_to_next_pose = pose_distance * (1.0 - current_progress);
                double remaining_angular_distance_to_next_pose = pose_distance_angular * (1.0 - current_progress);

                if (remaining_distance_to_next_pose < distance_to_move &&
                    remaining_angular_distance_to_next_pose < angle_to_move)
                {
                    // we need to move further than the remaining distance_to_move. Skip to the next point and decrease distance_to_move.
                    current_progress = 0.0;
                    current_index++;
                    distance_to_move -= remaining_distance_to_next_pose;
                    angle_to_move -= remaining_angular_distance_to_next_pose;
                }
                else
                {
                    // we cannot reach the next point yet, so we update the percentage
                    double current_progress_distance =
                        (pose_distance * current_progress + distance_to_move) / pose_distance;
                    double current_progress_angle =
                        (pose_distance_angular * current_progress + angle_to_move) / pose_distance_angular;
                    current_progress = fmin(current_progress_angle, current_progress_distance);
                    if (current_progress > 1.0)
                    {
                        ROS_WARN_STREAM("FTCLocalPlannerROS: FTC PLANNER: Progress > 1.0");
                        //                    current_progress = 1.0;
                    }
                    distance_to_move = 0;
                    angle_to_move = 0;
                }
            }

            tf2::fromMsg(global_plan[current_index].pose, currentPose);
            tf2::fromMsg(global_plan[current_index + 1].pose, nextPose);
            // interpolate between points
            Eigen::Quaternion<double> rot1(currentPose.linear());
            Eigen::Quaternion<double> rot2(nextPose.linear());

            Eigen::Vector3d trans1 = currentPose.translation();
            Eigen::Vector3d trans2 = nextPose.translation();

            Eigen::Affine3d result;
            result.translation() = (1.0 - current_progress) * trans1 + current_progress * trans2;
            result.linear() = rot1.slerp(current_progress, rot2).toRotationMatrix();

            current_control_point = result;
        }
        break;
        case POST_ROTATE:
            tf2::fromMsg(global_plan[global_plan.size() - 1].pose, current_control_point);
            break;
        case WAITING_FOR_GOAL_APPROACH:
            break;
        case FINISHED:
            break;
        }

        {
            geometry_msgs::PoseStamped viz;
            viz.header = global_plan[current_index].header;
            viz.pose = tf2::toMsg(current_control_point);
            global_point_pub.publish(viz);
        }
        auto map_to_base = tf_buffer->lookupTransform("base_link", "map", ros::Time(), ros::Duration(1.0));
        tf2::doTransform(current_control_point, local_control_point, map_to_base);

        lat_error = local_control_point.translation().y();
        lon_error = local_control_point.translation().x();
        angle_error = local_control_point.rotation().eulerAngles(0, 1, 2).z();
    }

    void FTCPlanner::calculate_velocity_commands(double dt, geometry_msgs::TwistStamped &cmd_vel)
    {
        // check, if we're completely done
        if (current_state == FINISHED || is_crashed)
        {
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.angular.z = 0;
            return;
        }

        i_lon_error += lon_error * dt;
        i_lat_error += lat_error * dt;
        i_angle_error += angle_error * dt;

        if (i_lon_error > config.ki_lon_max)
        {
            i_lon_error = config.ki_lon_max;
        }
        else if (i_lon_error < -config.ki_lon_max)
        {
            i_lon_error = -config.ki_lon_max;
        }
        if (i_lat_error > config.ki_lat_max)
        {
            i_lat_error = config.ki_lat_max;
        }
        else if (i_lat_error < -config.ki_lat_max)
        {
            i_lat_error = -config.ki_lat_max;
        }
        if (i_angle_error > config.ki_ang_max)
        {
            i_angle_error = config.ki_ang_max;
        }
        else if (i_angle_error < -config.ki_ang_max)
        {
            i_angle_error = -config.ki_ang_max;
        }

        double d_lat = (lat_error - last_lat_error) / dt;
        double d_lon = (lon_error - last_lon_error) / dt;
        double d_angle = (angle_error - last_angle_error) / dt;

        last_lat_error = lat_error;
        last_lon_error = lon_error;
        last_angle_error = angle_error;

        // allow linear movement only if in following state

        if (current_state == FOLLOWING)
        {
            double lin_speed = lon_error * config.kp_lon + i_lon_error * config.ki_lon + d_lon * config.kd_lon;
            if (lin_speed < 0 && config.forward_only)
            {
                lin_speed = 0;
            }
            else
            {
                if (lin_speed > config.max_cmd_vel_speed)
                {
                    lin_speed = config.max_cmd_vel_speed;
                }
                else if (lin_speed < -config.max_cmd_vel_speed)
                {
                    lin_speed = -config.max_cmd_vel_speed;
                }

                if (lin_speed < 0)
                {
                    lat_error *= -1.0;
                }
            }
            cmd_vel.twist.linear.x = lin_speed;
        }
        else
        {
            cmd_vel.twist.linear.x = 0.0;
        }

        if (current_state == FOLLOWING)
        {

            double ang_speed = angle_error * config.kp_ang + i_angle_error * config.ki_ang + d_angle * config.kd_ang +
                               lat_error * config.kp_lat + i_lat_error * config.ki_lat + d_lat * config.kd_lat;

            if (ang_speed > config.max_cmd_vel_ang)
            {
                ang_speed = config.max_cmd_vel_ang;
            }
            else if (ang_speed < -config.max_cmd_vel_ang)
            {
                ang_speed = -config.max_cmd_vel_ang;
            }

            cmd_vel.twist.angular.z = ang_speed;
        }
        else
        {
            double ang_speed = angle_error * config.kp_ang + i_angle_error * config.ki_ang + d_angle * config.kd_ang;
            if (ang_speed > config.max_cmd_vel_ang)
            {
                ang_speed = config.max_cmd_vel_ang;
            }
            else if (ang_speed < -config.max_cmd_vel_ang)
            {
                ang_speed = -config.max_cmd_vel_ang;
            }

            // Phase 3: obstacle-aware rotation direction override
            if (config.obstacle_aware_rotation && (current_state == PRE_ROTATE || current_state == POST_ROTATE))
            {
                int preferred_sign = (current_state == PRE_ROTATE) ? preferred_pre_rotate_sign_ : preferred_post_rotate_sign_;
                if (preferred_sign != 0)
                {
                    bool going_wrong_way = (preferred_sign > 0 && ang_speed < 0) || (preferred_sign < 0 && ang_speed > 0);
                    if (going_wrong_way)
                    {
                        double min_ang = config.max_cmd_vel_ang * 0.3;
                        ang_speed = preferred_sign * std::max(std::abs(ang_speed), min_ang);
                    }
                }
            }

            cmd_vel.twist.angular.z = ang_speed;

            // check if robot oscillates
            bool is_oscillating = checkOscillation(cmd_vel);
            if (is_oscillating)
            {
                ang_speed = config.max_cmd_vel_ang;
                cmd_vel.twist.angular.z = ang_speed;
            }
        }

        if (config.debug_pid)
        {
            ftc_local_planner::PID debugPidMsg;
            debugPidMsg.kp_lon_set = lon_error;

            // proportional
            debugPidMsg.kp_lat_set = lat_error * config.kp_lat;
            debugPidMsg.kp_lon_set = lon_error * config.kp_lon;
            debugPidMsg.kp_ang_set = angle_error * config.kp_ang;

            // integral
            debugPidMsg.ki_lat_set = i_lat_error * config.ki_lat;
            debugPidMsg.ki_lon_set = i_lon_error * config.ki_lon;
            debugPidMsg.ki_ang_set = i_angle_error * config.ki_ang;

            // diff
            debugPidMsg.kd_lat_set = d_lat * config.kd_lat;
            debugPidMsg.kd_lon_set = d_lon * config.kd_lon;
            debugPidMsg.kd_ang_set = d_angle * config.kd_ang;

            // errors
            debugPidMsg.lon_err = lon_error;
            debugPidMsg.lat_err = lat_error;
            debugPidMsg.ang_err = angle_error;

            // speeds
            debugPidMsg.ang_speed = cmd_vel.twist.angular.z;
            debugPidMsg.lin_speed = cmd_vel.twist.linear.x;

            pubPid.publish(debugPidMsg);
        }
    }

    bool FTCPlanner::getProgress(PlannerGetProgressRequest &req, PlannerGetProgressResponse &res)
    {
        res.index = current_index;
        return true;
    }

    bool FTCPlanner::checkCollision(int max_points)
    {
        unsigned int x;
        unsigned int y;

        std::vector<geometry_msgs::Point> footprint;
        visualization_msgs::Marker obstacle_marker;

        if (!config.check_obstacles)
        {
            return false;
        }
        // maximal costs
        unsigned char previous_cost = 255;
        // ensure look ahead not out of plan
        if (global_plan.size() < max_points)
        {
            max_points = global_plan.size();
        }

        // calculate cost of footprint at robots actual pose
        if (config.obstacle_footprint)
        {
        costmap->getOrientedFootprint(footprint);
        for (int i = 0; i < footprint.size(); i++)
        {
            // check cost of each point of footprint
            if (costmap_map_->worldToMap(footprint[i].x, footprint[i].y, x, y))
            {
                unsigned char costs = costmap_map_->getCost(x, y);
                if (costs >= costmap_2d::LETHAL_OBSTACLE)
                {
                    ROS_WARN("FTCLocalPlannerROS: Possible collision of footprint at actual pose. Stop local planner.");
                    return true;
                }
            }
        }
        }

        for (int i = 0; i < max_points; i++)
        {
            geometry_msgs::PoseStamped x_pose;
            int index = current_index + i;
            if (index > global_plan.size())
            {
                index = global_plan.size();
            }
            x_pose = global_plan[index];

            if (costmap_map_->worldToMap(x_pose.pose.position.x, x_pose.pose.position.y, x, y))
            {
                unsigned char costs = costmap_map_->getCost(x, y);
                if (config.debug_obstacle)
                {
                    debugObstacle(obstacle_marker, x, y, costs, max_points);
                }
                // Near at obstacel
                if (costs > 0)
                {
                    // Possible collision
                    if (costs > 127 && costs > previous_cost)
                    {
                        ROS_WARN("FTCLocalPlannerROS: Possible collision. Stop local planner.");
                        return true;
                    }
                }
                previous_cost = costs;
            }
        }
        return false;
    }

    bool FTCPlanner::checkOscillation(geometry_msgs::TwistStamped &cmd_vel)
    {
        bool oscillating = false;
        // detect and resolve oscillations
        if (config.oscillation_recovery)
        {
            // oscillating = true;
            double max_vel_theta = config.max_cmd_vel_ang;
            double max_vel_current = config.max_cmd_vel_speed;

            failure_detector_.update(cmd_vel, config.max_cmd_vel_speed, config.max_cmd_vel_speed, max_vel_theta,
                                     config.oscillation_v_eps, config.oscillation_omega_eps);

            oscillating = failure_detector_.isOscillating();

            if (oscillating) // we are currently oscillating
            {
                if (!oscillation_detected_) // do we already know that robot oscillates?
                {
                    time_last_oscillation_ = ros::Time::now(); // save time when oscillation was detected
                    oscillation_detected_ = true;
                }
                // calculate duration of actual oscillation
                bool oscillation_duration_timeout = !((ros::Time::now() - time_last_oscillation_).toSec() < config.oscillation_recovery_min_duration); // check how long we oscillate
                if (oscillation_duration_timeout)
                {
                    if (!oscillation_warning_) // ensure to send warning just once instead of spamming around
                    {
                        ROS_WARN("FTCLocalPlannerROS: possible oscillation (of the robot or its local plan) detected. Activating recovery strategy (prefer current turning direction during optimization).");
                        oscillation_warning_ = true;
                    }
                    return true;
                }
                return false; // oscillating but timeout not reached
            }
            else
            {
                // not oscillating
                time_last_oscillation_ = ros::Time::now(); // save time when oscillation was detected
                oscillation_detected_ = false;
                oscillation_warning_ = false;
                return false;
            }
        }
        return false; // no check for oscillation
    }

    void FTCPlanner::debugObstacle(visualization_msgs::Marker &obstacle_points, double x, double y, unsigned char cost, int maxIDs)
    {
        if (obstacle_points.points.empty())
        {
            obstacle_points.header.frame_id = costmap->getGlobalFrameID();
            obstacle_points.header.stamp = ros::Time::now();
            obstacle_points.action = visualization_msgs::Marker::ADD;
            obstacle_points.pose.orientation.w = 1.0;
            obstacle_points.type = visualization_msgs::Marker::POINTS;
            obstacle_points.scale.x = 0.2;
            obstacle_points.scale.y = 0.2;
        }
        obstacle_points.id = obstacle_points.points.size() + 1;

        if (cost < 127)
        {
            obstacle_points.color.g = 1.0f;
        }

        if (cost >= 127 && cost < 255)
        {
            obstacle_points.color.r = 1.0f;
        }
        obstacle_points.color.a = 1.0;
        geometry_msgs::Point p;
        costmap_map_->mapToWorld(x, y, p.x, p.y);
        p.z = 0;

        obstacle_points.points.push_back(p);
        if (obstacle_points.points.size() >= maxIDs || cost > 0)
        {
            obstacle_marker_pub.publish(obstacle_points);
            obstacle_points.points.clear();
        }
    }
}
